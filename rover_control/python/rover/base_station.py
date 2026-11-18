"""Base station: read a gamepad and send commands to the rover with CommandClient.

    rover-base                                    # gamepad -> rover PC at 10.10.10.5
    rover-base --rover-ip 127.0.0.1 --fake        # local test, scripted input
    rover-base --serial COM5                      # over an RFD900 serial modem
    rover-base --rover-ip 127.0.0.1 --fake --release   # also clear a latched e-stop
    rover-base --rover-ip 127.0.0.1 --fake --mode autonomous   # start in another mode
    rover-base --serial COM9 --show-sent          # print every packet as it goes out

Packets are binary, so a plain serial monitor on the far end shows '?' symbols.
Use --show-sent here, and run rover-node on the far end to see them decoded.
If the far end can only be a plain terminal, --text sends readable lines instead
(debug only: the rover does not drive from them):
    rover-base --serial COM9 --text               # far terminal shows DRIVE linear=+0.30 ...

Default gamepad mapping (Xbox layout; verify with --print-axes, it varies by OS):
    Left stick Y  -> linear (forward/back)   Right stick X -> turn
    B             -> E-STOP (latches)        START         -> release e-stop
    A             -> Teleop mode             X             -> Autonomous mode
    BACK          -> Idle mode
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from types import SimpleNamespace

from . import (ROVER_PC_IP, DEFAULT_BASE_PORT, DEFAULT_ROVER_PORT, DriveCmd, EStopCmd,
               Heartbeat, Mode, SetModeCmd, decode, shape_axis)
from .client import connect
from .link import SerialLink, UdpLink

SEND_HZ = 20
MODES = {"idle": Mode.Idle, "teleop": Mode.Teleop,
         "autonomous": Mode.Autonomous, "arrived": Mode.Arrived}

# Gamepad layout. Agree on this with the drive team: their xbox_controller_node
# steers with the LEFT stick X (axis 0) instead.
# Axis numbers differ by OS for Xbox controllers:
#   Windows (XInput): 0 LX, 1 LY, 2 RX, 3 RY, 4 LT, 5 RT
#   Linux (xpad):     0 LX, 1 LY, 2 LT, 3 RX, 4 RY, 5 RT
# Check yours with `rover-base --print-axes`.
AXIS_LINEAR = 1
AXIS_TURN = 3      # XInput backend (forced below): 0 LX, 1 LY, 2 LT, 3 RX, 4 RY, 5 RT
BTN_A, BTN_B, BTN_X, BTN_BACK, BTN_START = 0, 1, 2, 6, 7

# Stick feel (TODO step 3: move to YAML config)
DEADBAND = 0.08
EXPO = 0.3
TURN_SCALE = 0.7      # turning is twitchy at full scale; 70% feels better


class FakeInput:
    """Scripted input so you can test the whole pipeline without a controller."""

    def __init__(self):
        self.t0 = time.monotonic()

    def poll(self):
        t = time.monotonic() - self.t0
        return 0.6 * math.sin(t * 0.5), 0.4 * math.sin(t * 0.2), []


class GamepadInput:
    def __init__(self, index: int = 0, print_axes: bool = False):
        import os
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        # No window -> Windows RawInput/WGI deliver nothing; plain XInput works in the background.
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        os.environ.setdefault("SDL_JOYSTICK_RAWINPUT", "0")
        os.environ.setdefault("SDL_JOYSTICK_WGI", "0")
        import pygame
        self.pg = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise SystemExit(
                "No gamepad found. Plug the controller in (a USB cable that carries data, not "
                "charge-only) or pair it over Bluetooth, check it shows up in Windows under "
                "Settings -> Bluetooth & devices, then run this again. Or use --fake.")
        self.js = pygame.joystick.Joystick(index)
        self.js.init()
        self.print_axes = print_axes
        print(f"Using gamepad: {self.js.get_name()}")

    def poll(self):
        """Returns (linear, angular, actions). angular uses ROS sign: + = turn left."""
        actions = []
        for ev in self.pg.event.get():
            if ev.type == self.pg.JOYBUTTONDOWN:
                if self.print_axes:
                    print(f"button {ev.button}")
                if ev.button == BTN_B:
                    actions.append("estop")
                elif ev.button == BTN_START:
                    actions.append("release")
                elif ev.button == BTN_A:
                    actions.append(Mode.Teleop)
                elif ev.button == BTN_X:
                    actions.append(Mode.Autonomous)
                elif ev.button == BTN_BACK:
                    actions.append(Mode.Idle)
        linear = -self.js.get_axis(AXIS_LINEAR)   # stick up is negative in SDL
        angular = -self.js.get_axis(AXIS_TURN)    # stick right is positive; ROS: right = negative
        if self.print_axes:
            axes = [round(self.js.get_axis(i), 2) for i in range(self.js.get_numaxes())]
            print(f"axes {axes}")
        return (shape_axis(linear, DEADBAND, EXPO),
                shape_axis(angular, DEADBAND, EXPO, TURN_SCALE),
                actions)


def describe_packet(data: bytes) -> str:
    """Decode an outgoing packet and describe it in plain text.

    Decoding our own bytes checks the header, length and CRC, so 'OK' means the
    rover will accept exactly the values shown."""
    hex_bytes = data.hex(" ")
    pkt = decode(data)
    if pkt is None:
        return f"BAD PACKET (would be rejected) {len(data)} bytes: {hex_bytes}"
    msg = pkt.msg
    if isinstance(msg, DriveCmd):
        what = f"Drive   linear={msg.linear:+.2f} angular={msg.angular:+.2f}"
    elif isinstance(msg, EStopCmd):
        what = "E-STOP  engage" if msg.engage else "E-STOP  release"
    elif isinstance(msg, SetModeCmd):
        what = f"Mode    {msg.mode.name}"
    elif isinstance(msg, Heartbeat):
        what = "Heartbeat"
    else:
        what = type(msg).__name__
    return f"seq={pkt.seq:<5} {what:<40} OK  {len(data)} bytes: {hex_bytes}"


class SentPrinter:
    """on_send hook for --show-sent. Drive packets go out at 20 Hz, so only a few
    per second are printed; mode/e-stop packets are always printed."""

    def __init__(self, drive_hz: float = 4.0):
        self.period = 1.0 / drive_hz
        self.last_drive = 0.0

    def __call__(self, data: bytes) -> None:
        pkt = decode(data)
        if pkt is not None and isinstance(pkt.msg, DriveCmd):
            now = time.monotonic()
            if now - self.last_drive < self.period:
                return
            self.last_drive = now
        print(f"[sent] {describe_packet(data)}")


def do_action(rover, action) -> None:
    if action == "estop":
        rover.estop()
        print("-> E-STOP")
    elif action == "release":
        rover.release_estop()
        print("-> e-stop released")
    elif isinstance(action, Mode):
        rover.set_mode(action)
        print(f"-> mode {action.name}")


class TextClient:
    """--text debug mode: sends readable lines instead of binary packets.

    For checking the controller over a radio when the far end is only a serial
    terminal (e.g. the Arduino Serial Monitor), which shows binary packets as '?'.
    The rover, rover-node and the ROS bridge IGNORE these lines, so nothing drives.
    Has the same methods rover-base uses on CommandClient, so the main loop is shared.

        DRIVE linear=+0.30 angular=+0.08     (drive_hz times per second)
        MODE Teleop / ESTOP ON / ESTOP OFF   (once, when you press the button)

    Text typed in the far terminal is printed here as '<- far: ...'.
    """

    def __init__(self, link, drive_hz: float = 5.0, echo: bool = False):
        self.link = link
        self.period = 1.0 / drive_hz
        self.echo = echo
        self.last_drive = -1e9
        self.rx = b""
        self.stats = SimpleNamespace(packets_sent=0, telemetry_received=0, corrupt=0)

    def _send(self, line: str) -> None:
        self.link.send_bytes((line + "\r\n").encode("ascii"))
        self.stats.packets_sent += 1
        if self.echo:
            print(f"[sent] {line}")

    def drive(self, linear: float, angular: float) -> None:
        now = time.monotonic()
        if now - self.last_drive >= self.period:
            self.last_drive = now
            self._send(f"DRIVE linear={linear:+.2f} angular={angular:+.2f}")

    def set_mode(self, mode: Mode) -> None:
        self._send(f"MODE {mode.name}")

    def estop(self) -> None:
        self._send("ESTOP ON")

    def release_estop(self) -> None:
        self._send("ESTOP OFF")

    def poll(self) -> int:
        data = self.link.recv_bytes()
        if not data:
            return 0
        self.rx += data
        *lines, self.rx = self.rx.replace(b"\r", b"").split(b"\n")
        for line in lines:
            if line:
                print(f"<- far: {line.decode('ascii', errors='replace')}")
        return len(lines)

    def link_ok(self) -> bool:
        return False      # no telemetry in text mode

    def telemetry(self):
        return None

    def close(self) -> None:
        self.link.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rover-ip", default=ROVER_PC_IP)
    ap.add_argument("--rover-port", type=int, default=DEFAULT_ROVER_PORT)
    ap.add_argument("--listen-port", type=int, default=DEFAULT_BASE_PORT)
    ap.add_argument("--serial", help="use a serial radio (e.g. RFD900) on this port instead of UDP")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--fake", action="store_true", help="scripted input, no gamepad")
    ap.add_argument("--mode", choices=list(MODES),
                    help="set the rover mode on startup (default with --fake: teleop)")
    ap.add_argument("--release", action="store_true",
                    help="release a latched e-stop on startup (use after Ctrl+C)")
    ap.add_argument("--print-axes", action="store_true", help="debug gamepad mapping")
    ap.add_argument("--show-sent", action="store_true",
                    help="print each outgoing packet decoded (values, CRC check, raw bytes)")
    ap.add_argument("--text", action="store_true",
                    help="DEBUG: send readable text lines instead of packets, for a plain serial "
                         "terminal on the far end. The rover will NOT drive in this mode.")
    ap.add_argument("--text-hz", type=float, default=5.0,
                    help="DRIVE lines per second in --text mode (default 5)")
    args = ap.parse_args(argv)

    # Gamepad first: if it is missing we exit before opening (and holding) the radio port.
    source = FakeInput() if args.fake else GamepadInput(print_axes=args.print_axes)
    try:
        if args.text:
            link = (SerialLink(args.serial, args.baud) if args.serial
                    else UdpLink(args.listen_port, peer=(args.rover_ip, args.rover_port)))
            rover = TextClient(link, drive_hz=args.text_hz, echo=args.show_sent)
        else:
            rover = connect(args.rover_ip, rover_port=args.rover_port,
                            listen_port=args.listen_port, serial=args.serial, baud=args.baud,
                            on_send=SentPrinter() if args.show_sent else None)
    except Exception as exc:   # serial.SerialException, or OSError for a busy UDP port
        if args.serial and "denied" in str(exc).lower():
            raise SystemExit(
                f"{args.serial} is already open in another program. Close it there first: "
                "Arduino Serial Monitor, a VS Code/Cursor serial monitor, PuTTY, RFD Modem Tools, "
                "or another rover-base window. If nothing else has it, unplug and replug the radio.")
        if args.serial:
            raise SystemExit(f"Could not open {args.serial}: {exc}\n"
                             "Check the port name in Device Manager -> Ports (COM & LPT).")
        raise
    target = args.serial or f"{args.rover_ip}:{args.rover_port}"
    if args.text:
        print(f"TEXT DEBUG MODE: sending readable lines to {target} at {args.text_hz:g} Hz. "
              "The rover will not drive from these. Ctrl+C quits.")
    else:
        print(f"Sending to {target} at {SEND_HZ} Hz. Ctrl+C sends E-STOP and quits.")

    if args.release:
        do_action(rover, "release")
    start_mode = args.mode or ("teleop" if args.fake else None)
    if start_mode:
        do_action(rover, MODES[start_mode])

    period = 1.0 / SEND_HZ
    last_status = 0.0
    link_was_ok = False
    try:
        next_tick = time.monotonic()
        while True:
            linear, angular, actions = source.poll()
            for action in actions:
                do_action(rover, action)
            rover.drive(linear, angular)        # also keeps the link alive

            rover.poll()
            now = time.monotonic()
            link_ok = rover.link_ok()
            if link_ok != link_was_ok:
                print("<- rover link UP" if link_ok else "<- rover link DOWN (no telemetry)")
                link_was_ok = link_ok
            t = rover.telemetry()
            if t and link_ok and now - last_status > 1.0:
                last_status = now
                print(f"<- rover mode={t.mode.name} estop={t.estop} "
                      f"linear={t.linear:+.2f} angular={t.angular:+.2f} batt={t.battery_v:.1f}V")

            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\nSending E-STOP and exiting.")
        rover.estop()
    finally:
        s = rover.stats
        print(f"sent={s.packets_sent} telemetry={s.telemetry_received} corrupt={s.corrupt}")
        rover.close()


if __name__ == "__main__":
    main()

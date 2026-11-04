"""Base station: read a gamepad and send commands to the rover with CommandClient.

    rover-base                                    # gamepad -> rover PC at 10.10.10.5
    rover-base --rover-ip 127.0.0.1 --fake        # local test, scripted input
    rover-base --serial COM5                      # over an RFD900 serial modem
    rover-base --rover-ip 127.0.0.1 --fake --release   # also clear a latched e-stop
    rover-base --rover-ip 127.0.0.1 --fake --mode autonomous   # start in another mode

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

from . import ROVER_PC_IP, DEFAULT_BASE_PORT, DEFAULT_ROVER_PORT, Mode, shape_axis
from .client import connect

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
AXIS_TURN = 2 if sys.platform == "win32" else 3
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
        import pygame
        self.pg = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise SystemExit("No gamepad found. Plug one in or use --fake.")
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
    args = ap.parse_args(argv)

    rover = connect(args.rover_ip, rover_port=args.rover_port, listen_port=args.listen_port,
                    serial=args.serial, baud=args.baud)
    source = FakeInput() if args.fake else GamepadInput(print_axes=args.print_axes)
    target = args.serial or f"{args.rover_ip}:{args.rover_port}"
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

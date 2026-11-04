"""Virtual rover: receives commands, runs the C++ safety gate, prints what it would
send to the drive system. Use it to test the base station / CommandClient on a laptop.

On the real rover this job will be done by a ROS 2 node in the `comms` package,
which publishes the same output to /drive/cmd_vel for the drive team's controller.

    rover-node                                  # UDP (Groove link / localhost)
    rover-node --radio-serial /dev/ttyUSB0      # commands via RFD900 instead of UDP
    rover-node --demo-autonomy                  # pretend autonomy stack drives in Autonomous
"""
from __future__ import annotations

import argparse
import time

from . import (DEFAULT_BASE_PORT, DEFAULT_ROVER_PORT, ControllerConfig, DriveOutput, Mode,
               RoverController)
from .link import SerialLink, UdpLink

CONTROL_HZ = 50
TELEMETRY_HZ = 5
PRINT_HZ = 5


def describe(out: DriveOutput) -> str:
    state = "DRIVE" if out.enabled else f"STOP({out.reason.name})"
    return (f"[cmd_vel] linear={out.linear:+.2f} angular={out.angular:+.2f} "
            f"{state:<20} LED={out.led.name}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=DEFAULT_ROVER_PORT)
    ap.add_argument("--base-port", type=int, default=DEFAULT_BASE_PORT)
    ap.add_argument("--radio-serial", help="serial radio (RFD900) port; default is UDP")
    ap.add_argument("--baud", type=int, default=57600, help="serial radio baud")
    ap.add_argument("--demo-autonomy", action="store_true",
                    help="fake autonomy stack: creep forward at 30%% while in Autonomous mode")
    ap.add_argument("--timeout", type=float, default=0.5, help="link-loss timeout, seconds")
    args = ap.parse_args(argv)

    cfg = ControllerConfig()
    cfg.watchdog_timeout_s = args.timeout
    ctrl = RoverController(cfg)

    serial_radio = bool(args.radio_serial)
    link = SerialLink(args.radio_serial, args.baud) if serial_radio else UdpLink(args.port)
    period = 1.0 / CONTROL_HZ
    last_tel = last_print = 0.0
    base_addr = None
    where = args.radio_serial if serial_radio else f"UDP {args.port}"
    print(f"Virtual rover listening on {where}, loop {CONTROL_HZ} Hz. Ctrl+C to quit.")

    try:
        next_tick = time.monotonic()
        while True:
            now = time.monotonic()
            if serial_radio:
                for pkt in link.recv_packets():
                    ctrl.handle_packet(pkt, now)
            else:
                for data, addr in link.recv_raw():
                    if ctrl.handle_bytes(data, now):
                        base_addr = (addr[0], args.base_port)   # reply to whoever is driving

            # Autonomy hook: the real nav stack will publish /autonomy/cmd_vel.
            if args.demo_autonomy and ctrl.mode == Mode.Autonomous:
                ctrl.set_autonomy_command(0.3, 0.0, now)

            out = ctrl.update(now)
            if now - last_print >= 1.0 / PRINT_HZ:
                last_print = now
                print(describe(out))

            if (serial_radio or base_addr) and now - last_tel >= 1.0 / TELEMETRY_HZ:
                last_tel = now
                if not serial_radio:
                    link.peer = base_addr
                link.send(ctrl.make_telemetry(0, 0.0).msg)   # send() sets the seq number

            next_tick += period
            sleep = next_tick - time.monotonic()
            if sleep < -period * 5:      # fell way behind (e.g. laptop suspended): resync
                next_tick = time.monotonic()
            time.sleep(max(0.0, sleep))
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        link.close()
        s = ctrl.stats
        print(f"link stats: accepted={s.accepted} corrupt={s.corrupt} stale={s.stale}")


if __name__ == "__main__":
    main()

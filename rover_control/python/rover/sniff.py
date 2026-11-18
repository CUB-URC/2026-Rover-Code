"""rover-sniff: show, in plain text, what is arriving on a serial radio. Receive only.

    rover-sniff --serial COM9                 # RFD900 on this PC (default 57600 baud)
    rover-sniff --serial COM9 --all           # print every frame, not ~10 per second
    rover-sniff --serial COM9 --hex           # also show the raw bytes of each frame

A plain serial monitor shows controller data as '?' because it is binary. This
decodes the formats the team uses, from the same byte stream:

  * Jackson's sticks frames (ros2_ws/src/comms/src/comms_node.cpp):
        [FF][01][var1][var2][0A]     each var byte 0..254 -> -1.0 .. +1.0
  * rover_control packets (rover-base):  [AB][CD][ver][type][seq][len][payload][crc]
  * plain text lines (e.g. typed into an Arduino Serial Monitor)

It never transmits, so it cannot disturb the sender or the radios.
"""
from __future__ import annotations

import argparse
import time
from typing import List, Optional, Tuple

from . import DriveCmd, EStopCmd, SetModeCmd, StreamDecoder

STICKS_START = 0xFF
STICKS_TYPE = 0x01          # PacketType::STICKS
STICKS_END = 0x0A           # '\n'


def byte_to_float(value: int) -> float:
    """Same mapping as comms_node.cpp: 0 -> -1.0, 127 -> 0.0, 254 -> +1.0."""
    return (value / 254.0) * 2.0 - 1.0


class SticksParser:
    """Python copy of PacketParser in comms_node.cpp, state for state, so this
    tool accepts exactly the frames the rover's comms_node would accept."""

    def __init__(self):
        self.state = 0
        self.var1 = 0
        self.var2 = 0
        self.frames = 0

    def push(self, data: bytes) -> List[Tuple[int, int]]:
        out = []
        for b in data:
            if self.state == 0:
                self.state = 1 if b == STICKS_START else 0
            elif self.state == 1:
                if b == STICKS_TYPE:
                    self.state = 2
                else:
                    self.state = 1 if b == STICKS_START else 0
            elif self.state == 2:
                self.var1 = b
                self.state = 3
            elif self.state == 3:
                self.var2 = b
                self.state = 4
            else:  # state 4
                if b == STICKS_END:
                    out.append((self.var1, self.var2))
                    self.frames += 1
                    self.state = 0
                else:
                    self.state = 1 if b == STICKS_START else 0
        return out


class TextLines:
    """Collects readable lines. A line containing a 0xFF or 0xAB byte is part of
    a binary frame (sticks or rover_control), not text, so it is dropped."""

    def __init__(self):
        self.buf = bytearray()

    def push(self, data: bytes) -> List[str]:
        out = []
        for b in data:
            if b == 0x0A:
                line = bytes(self.buf).rstrip(b"\r")
                self.buf.clear()
                # >= 2 chars: a sticks value byte that happens to be 0x0A splits a
                # frame and leaves a 1-byte tail that is not text.
                if (len(line) >= 2 and 0xFF not in line and 0xAB not in line
                        and all(32 <= c < 127 or c == 9 for c in line)):
                    out.append(line.decode("ascii"))
            elif not (32 <= b < 127 or b in (9, 13)):
                self.buf.clear()            # binary byte: text (if any) starts after it
            else:
                self.buf.append(b)
                if len(self.buf) > 200:     # no newline in a long time: not text
                    self.buf.clear()
        return out


def describe_rover_packet(pkt) -> str:
    msg = pkt.msg
    if isinstance(msg, DriveCmd):
        return f"Drive linear={msg.linear:+.2f} angular={msg.angular:+.2f}"
    if isinstance(msg, EStopCmd):
        return "E-STOP engage" if msg.engage else "E-STOP release"
    if isinstance(msg, SetModeCmd):
        return f"Mode {msg.mode.name}"
    return type(msg).__name__


class Sniffer:
    def __init__(self, print_all: bool = False, show_hex: bool = False, max_hz: float = 10.0):
        self.sticks = SticksParser()
        self.rover = StreamDecoder()
        self.text = TextLines()
        self.print_all = print_all
        self.show_hex = show_hex
        self.period = 0.0 if print_all else 1.0 / max_hz
        self.last_print = {"sticks": -1e9, "drive": -1e9}
        self.bytes_in = 0
        self.rover_packets = 0
        self.text_lines = 0

    def _due(self, kind: str, now: float) -> bool:
        if now - self.last_print[kind] >= self.period:
            self.last_print[kind] = now
            return True
        return False

    def feed(self, data: bytes, now: Optional[float] = None) -> List[str]:
        """Decode a chunk of bytes; returns the lines to print."""
        now = time.monotonic() if now is None else now
        self.bytes_in += len(data)
        lines = []
        # One byte at a time so a frame ending mid-chunk also resets the text
        # buffer (a packet's CRC bytes can look like letters). Radio rates are a
        # few kB/s at most, so this costs nothing.
        for i in range(len(data)):
            b = data[i:i + 1]
            frame_ended = False
            for v1, v2 in self.sticks.push(b):
                frame_ended = True
                if self._due("sticks", now):
                    s = (f"[sticks] var1={byte_to_float(v1):+.2f} var2={byte_to_float(v2):+.2f}"
                         f"   (raw {v1:3d}, {v2:3d})")
                    if self.show_hex:
                        s += f"   bytes: ff 01 {v1:02x} {v2:02x} 0a"
                    lines.append(s)
            for pkt in self.rover.push(b):
                frame_ended = True
                self.rover_packets += 1
                if isinstance(pkt.msg, DriveCmd) and not self._due("drive", now):
                    continue
                lines.append(f"[rover_control] seq={pkt.seq} {describe_rover_packet(pkt)}")
            if frame_ended:              # this byte closed a binary frame: not text
                self.text.buf.clear()
                continue
            for t in self.text.push(b):
                self.text_lines += 1
                lines.append(f"[text] {t}")
        return lines

    def summary(self, seconds: float) -> str:
        rate = self.sticks.frames / seconds if seconds > 0 else 0.0
        return (f"--- {self.bytes_in} bytes: {self.sticks.frames} sticks frames ({rate:.1f}/s), "
                f"{self.rover_packets} rover_control packets, {self.text_lines} text lines, "
                f"{self.rover.corrupt} bad rover_control frames")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", required=True, help="serial port of the radio, e.g. COM9")
    ap.add_argument("--baud", type=int, default=57600,
                    help="serial speed of THIS radio (RFD900 setting S1), default 57600")
    ap.add_argument("--all", action="store_true", help="print every frame (default ~10/s)")
    ap.add_argument("--hex", action="store_true", help="show the raw bytes of each sticks frame")
    args = ap.parse_args(argv)

    import serial
    try:
        port = serial.Serial(args.serial, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        if "denied" in str(exc).lower():
            raise SystemExit(
                f"{args.serial} is already open in another program (a serial monitor, "
                "rover-base, rover-node...). Close it there first.")
        raise SystemExit(f"Could not open {args.serial}: {exc}")

    sniffer = Sniffer(print_all=args.all, show_hex=args.hex)
    print(f"Listening on {args.serial} at {args.baud} baud (receive only). Ctrl+C to quit.")
    t0 = last_summary = time.monotonic()
    try:
        while True:
            data = port.read(port.in_waiting or 1)
            for line in sniffer.feed(data):
                print(line)
            now = time.monotonic()
            if now - last_summary >= 5.0:
                last_summary = now
                if sniffer.bytes_in == 0:
                    print("--- nothing received yet: is the sender running and are the radios linked?")
                else:
                    print(sniffer.summary(now - t0))
    except KeyboardInterrupt:
        print()
    finally:
        print(sniffer.summary(time.monotonic() - t0))
        port.close()


if __name__ == "__main__":
    main()

"""Transports: UDP (Groove radios / any network) and serial (RFD900).
Both offer send_bytes()/recv_bytes() for CommandClient, and send()/recv for the
virtual rover. UDP transport notes: UDP suits a lossy radio link: a late drive command is
useless, so we never wait for retransmits - the next packet (20-50 ms later)
replaces it, and the watchdog covers longer outages."""
from __future__ import annotations

import socket
from typing import Iterator, Optional, Tuple

from . import Packet, StreamDecoder, decode, encode

# DSCP "Expedited Forwarding": asks the radios to queue control packets ahead of
# bulk traffic. Matters because video shares the 2.4 GHz Groove link. RouterOS
# only honours it if a queue/WMM policy is configured, but marking costs nothing.
DSCP_EF_TOS = 0xB8


class UdpLink:
    """Used over the MikroTik Groove 52 ac bridge (or any IP network)."""

    def __init__(self, bind_port: int, bind_host: str = "0.0.0.0",
                 peer: Optional[Tuple[str, int]] = None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, DSCP_EF_TOS)
        except (AttributeError, OSError):
            pass  # not supported on every OS (e.g. some Windows setups)
        if hasattr(socket, "SIO_UDP_CONNRESET"):
            # Windows quirk: if we send to a port nobody is listening on (e.g. the
            # base station just closed), the next recvfrom() raises WinError 10054.
            # This turns that behaviour off.
            self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        self.sock.bind((bind_host, bind_port))
        self.sock.setblocking(False)
        self.peer = peer
        self._seq = 0

    def next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def send(self, msg) -> None:
        """Encode and send one message (used by the virtual rover)."""
        self.send_bytes(encode(Packet(self.next_seq(), msg)))

    def send_bytes(self, data: bytes) -> None:
        """Send one already-encoded packet (used by CommandClient)."""
        if self.peer is None:
            return
        try:
            self.sock.sendto(data, self.peer)
        except OSError as exc:  # e.g. network unreachable: keep the control loop alive
            print(f"[link] send failed: {exc}")

    def recv_bytes(self) -> bytes:
        """Return one waiting datagram, or b'' if nothing is waiting (used by CommandClient)."""
        for data, _ in self.recv_raw():
            return data
        return b""

    def recv_raw(self) -> Iterator[Tuple[bytes, Tuple[str, int]]]:
        """Yield every datagram waiting in the socket buffer (non-blocking)."""
        while True:
            try:
                data, addr = self.sock.recvfrom(256)
            except (BlockingIOError, InterruptedError):
                return
            except ConnectionResetError:
                continue  # peer went away; the watchdog handles the stop
            yield data, addr

    def recv_packets(self) -> Iterator[Packet]:
        for data, _ in self.recv_raw():
            pkt = decode(data)
            if pkt is not None:
                yield pkt

    def close(self) -> None:
        self.sock.close()


class SerialLink:
    """For serial radio modems such as the RFD900 (900 MHz) in the comms plan.
    Same send() interface as UdpLink. Frames are only ~10-22 bytes, so 20 Hz of
    drive commands is ~400 B/s - well within an RFD900's air data rate.
    Requires `pip install pyserial`."""

    def __init__(self, port: str, baud: int = 57600):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.05)
        self.decoder = StreamDecoder()
        self._seq = 0

    def next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def send(self, msg) -> None:
        self.send_bytes(encode(Packet(self.next_seq(), msg)))

    def send_bytes(self, data: bytes) -> None:
        try:
            self.ser.write(data)
        except Exception as exc:
            print(f"[link] serial write failed: {exc}")

    def recv_bytes(self) -> bytes:
        n = self.ser.in_waiting
        return self.ser.read(n) if n else b""

    def recv_packets(self) -> Iterator[Packet]:
        n = self.ser.in_waiting
        if n:
            yield from self.decoder.push(self.ser.read(n))

    @property
    def corrupt(self) -> int:
        return self.decoder.corrupt

    def close(self) -> None:
        self.ser.close()

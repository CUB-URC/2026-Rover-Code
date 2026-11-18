"""connect(): the one-liner for getting a CommandClient on a real link.

    from rover import connect, Mode

    rover = connect("127.0.0.1")              # UDP (Groove radios); default is 10.10.10.5
    rover = connect(serial="COM5")            # or an RFD900 serial modem
    rover.set_mode(Mode.Teleop)
    rover.drive(0.5, 0.0)                     # linear, angular (+ = left); ~20x per second
    rover.poll()                              # read telemetry
    print(rover.telemetry(), rover.link_ok())
    rover.estop()
    rover.close()
"""
from __future__ import annotations

from typing import Callable, Optional

from . import (DEFAULT_BASE_PORT, DEFAULT_ROVER_PORT, ROVER_PC_IP, ClientConfig,
               CommandClient)
from .link import SerialLink, UdpLink


def connect(rover_ip: str = ROVER_PC_IP, *, rover_port: int = DEFAULT_ROVER_PORT,
            listen_port: int = DEFAULT_BASE_PORT, serial: Optional[str] = None,
            baud: int = 57600, cfg: Optional[ClientConfig] = None,
            on_send: Optional[Callable[[bytes], None]] = None) -> CommandClient:
    """Open a link and return a CommandClient that uses it.

    UDP by default; pass serial="COM5" (or "/dev/ttyUSB0") for an RFD900 modem.
    The link is attached as client.link; call client.close() when done.
    on_send, if given, is called with every encoded packet just before it is sent
    (e.g. to print what is going out; see rover-base --show-sent).
    """
    if serial:
        link = SerialLink(serial, baud)
    else:
        link = UdpLink(listen_port, peer=(rover_ip, rover_port))

    send = link.send_bytes
    if on_send is not None:
        def send(data: bytes, _raw=link.send_bytes) -> None:
            on_send(data)
            _raw(data)

    client = CommandClient(send, link.recv_bytes, cfg or ClientConfig())
    client.link = link
    client.close = link.close
    return client

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

from typing import Optional

from . import (DEFAULT_BASE_PORT, DEFAULT_ROVER_PORT, ROVER_PC_IP, ClientConfig,
               CommandClient)
from .link import SerialLink, UdpLink


def connect(rover_ip: str = ROVER_PC_IP, *, rover_port: int = DEFAULT_ROVER_PORT,
            listen_port: int = DEFAULT_BASE_PORT, serial: Optional[str] = None,
            baud: int = 57600, cfg: Optional[ClientConfig] = None) -> CommandClient:
    """Open a link and return a CommandClient that uses it.

    UDP by default; pass serial="COM5" (or "/dev/ttyUSB0") for an RFD900 modem.
    The link is attached as client.link; call client.close() when done.
    """
    if serial:
        link = SerialLink(serial, baud)
    else:
        link = UdpLink(listen_port, peer=(rover_ip, rover_port))

    client = CommandClient(link.send_bytes, link.recv_bytes, cfg or ClientConfig())
    client.link = link
    client.close = link.close
    return client

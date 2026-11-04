"""Rover command link. The heavy lifting lives in C++ (rover._core)."""
from ._core import (  # noqa: F401
    ClientConfig, ClientStats, CommandClient, ControllerConfig, DriveCmd, DriveOutput,
    EStopCmd, Heartbeat, LedState, LinkStats, Mode, MsgType, Packet, RoverController,
    SetModeCmd, StopReason, StreamDecoder, Telemetry, Watchdog, apply_deadband, apply_expo,
    crc16_ccitt, decode, encode, led_for_mode, sanitize, shape_axis,
)

# CU Rover network (from the Comms Team "Groove 52 ac" setup doc):
#   radios: GS 10.10.10.1, Rover 10.10.10.2, Drone 10.10.10.3  (bridged, /24)
#   PCs:    GS 10.10.10.4, Rover 10.10.10.5, Drone 10.10.10.6
# TODO(step 3): move these into config/radios/link_config.yaml
ROVER_PC_IP = "10.10.10.5"
GS_PC_IP = "10.10.10.4"

DEFAULT_ROVER_PORT = 5005   # base station -> rover (commands)
DEFAULT_BASE_PORT = 5006    # rover -> base station (telemetry)

from .client import connect  # noqa: E402,F401

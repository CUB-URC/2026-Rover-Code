"""ROS-free core of the rover-side bridge.

Receives commands from the base station, runs them through the C++ RoverController
(e-stop, link-loss, modes) and returns the drive command that should go to
/drive/cmd_vel. Kept separate from ROS so it can be tested on any laptop;
ros/comms_bridge_node.py wraps it in a ROS 2 node.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from . import (DEFAULT_BASE_PORT, DEFAULT_ROVER_PORT, ControllerConfig, DriveOutput, EStopCmd,
               Packet, RoverController)
from .link import UdpLink


@dataclass
class BridgeOutput:
    linear: float          # -> Twist linear.x  (already scaled by max_linear)
    angular: float         # -> Twist angular.z (+ = turn left, already scaled)
    drive: DriveOutput     # full controller output (enabled, reason, led)


class BridgeCore:
    def __init__(self, port: int = DEFAULT_ROVER_PORT, base_port: int = DEFAULT_BASE_PORT,
                 cfg: Optional[ControllerConfig] = None, max_linear: float = 1.0,
                 max_angular: float = 1.0, telemetry_hz: float = 5.0):
        self.ctrl = RoverController(cfg or ControllerConfig())
        self.link = UdpLink(port)
        self.base_port = base_port
        self.base_addr: Optional[Tuple[str, int]] = None
        self.max_linear = max(0.0, min(1.0, max_linear))
        self.max_angular = max(0.0, min(1.0, max_angular))
        self.telemetry_period = 1.0 / telemetry_hz
        self._last_tel = -1e9
        self.battery_v = 0.0

    def autonomy_command(self, linear: float, angular: float, now: float) -> None:
        """From the autonomy stack (/autonomy/cmd_vel). Only used in Autonomous mode."""
        self.ctrl.set_autonomy_command(linear, angular, now)

    def local_estop(self, now: float) -> None:
        """E-stop from the rover side (e.g. a ROS service). Release only from the base station."""
        # An engage is always honoured, whatever its sequence number.
        self.ctrl.handle_packet(Packet(0, EStopCmd(True)), now)

    def step(self, now: float) -> BridgeOutput:
        """Call at a fixed rate (e.g. 50 Hz)."""
        for data, addr in self.link.recv_raw():
            if self.ctrl.handle_bytes(data, now):
                self.base_addr = (addr[0], self.base_port)   # reply to whoever is driving

        out = self.ctrl.update(now)

        if self.base_addr and now - self._last_tel >= self.telemetry_period:
            self._last_tel = now
            self.link.peer = self.base_addr
            self.link.send(self.ctrl.make_telemetry(0, self.battery_v).msg)

        if not out.enabled:
            return BridgeOutput(0.0, 0.0, out)
        return BridgeOutput(out.linear * self.max_linear, out.angular * self.max_angular, out)

    def close(self) -> None:
        self.link.close()

"""Motor backends. The controller produces left/right speeds in -1..1; a backend
turns them into something the hardware understands. Swap in your own once the
electrical team picks motor controllers (e.g. CAN for SparkMAX/ODrive/VESC)."""
from __future__ import annotations

import time

from . import LedState, MotorOutput


class MotorBackend:
    def write(self, out: MotorOutput) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        """Called on shutdown; must leave the motors stopped."""

    def battery_voltage(self) -> float:
        return 0.0


class SimBackend(MotorBackend):
    """Prints commands at a limited rate. Use for bench testing with no hardware."""

    def __init__(self, print_hz: float = 5.0):
        self._period = 1.0 / print_hz
        self._last = 0.0

    def write(self, out: MotorOutput) -> None:
        now = time.monotonic()
        if now - self._last < self._period:
            return
        self._last = now
        state = "DRIVE" if out.enabled else f"STOP({out.reason.name})"
        print(f"[motors] L={out.left:+.2f} R={out.right:+.2f} {state:<22} LED={out.led.name}")


class SerialBackend(MotorBackend):
    """Sends a line-based protocol to a microcontroller (see firmware/motor_bridge):

        M,<left>,<right>,<enabled>,<led>\\n     e.g.  M,0.500,-0.250,1,2

    The microcontroller runs its OWN watchdog, so if this computer crashes the
    motors still stop. Requires `pip install pyserial`."""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # imported lazily so the sim works without pyserial
        self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.05)

    def write(self, out: MotorOutput) -> None:
        line = (f"M,{out.left:.3f},{out.right:.3f},"
                f"{int(out.enabled)},{int(out.led)}\n")
        try:
            self.ser.write(line.encode("ascii"))
        except Exception as exc:  # serial timeouts must not kill the control loop
            print(f"[motors] serial write failed: {exc}")

    def stop(self) -> None:
        try:
            self.ser.write(f"M,0,0,0,{int(LedState.Off)}\n".encode("ascii"))
            self.ser.flush()
        finally:
            self.ser.close()

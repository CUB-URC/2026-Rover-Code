"""Python-side tests: prove the pybind11 bindings work. Run: python -m pytest tests"""
import time

import rover
from rover import (CommandClient, DriveCmd, EStopCmd, LedState, Mode, Packet, RoverController,
                   SetModeCmd, StopReason, Telemetry, connect, decode, encode)


# ---------------------------------------------------------------- protocol
def test_encode_decode_roundtrip():
    data = encode(Packet(5, DriveCmd(0.5, -0.5)))
    assert isinstance(data, bytes)
    pkt = decode(data)
    assert pkt.seq == 5 and isinstance(pkt.msg, DriveCmd)
    assert abs(pkt.msg.linear - 0.5) < 1e-6 and abs(pkt.msg.angular + 0.5) < 1e-6


def test_garbage_returns_none():
    assert decode(b"hello") is None
    assert decode(b"") is None


def test_stream_decoder_handles_split_frames():
    dec = rover.StreamDecoder()
    data = b"\x00junk" + encode(Packet(1, DriveCmd(0.2, 0.0))) + encode(Packet(2, EStopCmd(True)))
    got = []
    for i in range(0, len(data), 3):          # arrive in 3-byte chunks
        got += dec.push(data[i:i + 3])
    assert [p.seq for p in got] == [1, 2]


def test_shape_axis():
    assert rover.shape_axis(0.05, 0.08, 0.3) == 0.0
    assert abs(rover.shape_axis(1.0, 0.08, 0.3, 0.7) - 0.7) < 1e-6
    assert rover.sanitize(float("nan")) == 0.0


# ---------------------------------------------------------------- controller
def test_controller_drive_and_estop():
    c = RoverController()
    assert c.handle_bytes(encode(Packet(1, SetModeCmd(Mode.Teleop))), 0.0)
    c.handle_bytes(encode(Packet(2, DriveCmd(0.7, 0.2))), 0.0)
    out = c.update(0.02)
    assert out.enabled and abs(out.linear - 0.7) < 1e-6 and abs(out.angular - 0.2) < 1e-6
    assert out.led == LedState.Blue
    c.handle_bytes(encode(Packet(3, EStopCmd(True))), 0.03)
    out = c.update(0.04)
    assert not out.enabled and out.reason == StopReason.EStop and out.linear == 0.0


def test_link_loss_stops():
    c = RoverController()
    c.handle_packet(Packet(1, SetModeCmd(Mode.Teleop)), 0.0)
    c.handle_packet(Packet(2, DriveCmd(1.0, 0.0)), 0.0)
    assert c.update(1.0).reason == StopReason.LinkLost


# ---------------------------------------------------------------- command client
class FakeLink:
    def __init__(self):
        self.sent, self.inbox = [], []

    def send(self, data):
        self.sent.append(data)

    def recv(self):
        return self.inbox.pop(0) if self.inbox else b""


def test_client_functions_send_packets():
    link = FakeLink()
    client = CommandClient(link.send, link.recv)
    client.drive(0.5, 0.25)
    client.set_mode(Mode.Autonomous)       # critical: 3 copies
    client.estop()                         # critical: 3 copies
    client.release_estop()                 # critical: 3 copies
    client.stop()
    msgs = [decode(b).msg for b in link.sent]
    assert len(msgs) == 11
    assert isinstance(msgs[0], DriveCmd) and abs(msgs[0].angular - 0.25) < 1e-6
    assert all(isinstance(m, SetModeCmd) and m.mode == Mode.Autonomous for m in msgs[1:4])
    assert all(isinstance(m, EStopCmd) and m.engage for m in msgs[4:7])
    assert all(isinstance(m, EStopCmd) and not m.engage for m in msgs[7:10])
    assert isinstance(msgs[10], DriveCmd) and msgs[10].linear == 0.0
    assert [decode(b).seq for b in link.sent] == list(range(1, 12))
    assert client.stats.packets_sent == 11


def test_client_sanitizes_drive():
    link = FakeLink()
    client = CommandClient(link.send, link.recv)
    client.drive(float("nan"), 5.0)
    m = decode(link.sent[0]).msg
    assert m.linear == 0.0 and m.angular == 1.0


def test_client_reads_telemetry():
    link = FakeLink()
    client = CommandClient(link.send, link.recv)
    assert client.telemetry() is None and not client.link_ok()
    t = Telemetry()
    t.mode, t.linear = Mode.Teleop, 0.4
    link.inbox.append(encode(Packet(1, t)))
    assert client.poll() == 1
    assert client.telemetry().mode == Mode.Teleop and client.link_ok()


def test_end_to_end_over_localhost_udp():
    """CommandClient -> real UDP socket -> RoverController -> telemetry back."""
    from rover.link import UdpLink
    rover_link = UdpLink(15005)
    client = connect("127.0.0.1", rover_port=15005, listen_port=15006)
    ctrl = RoverController()
    try:
        client.set_mode(Mode.Teleop)
        client.drive(0.6, -0.1)
        time.sleep(0.05)
        now = time.monotonic()
        for data, addr in rover_link.recv_raw():
            ctrl.handle_bytes(data, now)
        out = ctrl.update(now)
        assert out.enabled and abs(out.linear - 0.6) < 1e-6 and abs(out.angular + 0.1) < 1e-6

        rover_link.peer = ("127.0.0.1", 15006)
        rover_link.send(ctrl.make_telemetry(0, 24.0).msg)
        time.sleep(0.05)
        assert client.poll() == 1
        assert client.telemetry().mode == Mode.Teleop
    finally:
        client.close()
        rover_link.close()


# ---------------------------------------------------------------- rover-side bridge core
def test_bridge_core_end_to_end():
    """CommandClient -> UDP -> BridgeCore: scaled cmd_vel, zero when stopped, telemetry back."""
    from rover.bridge_core import BridgeCore
    core = BridgeCore(port=15105, base_port=15106, max_linear=0.5, max_angular=0.25)
    client = connect("127.0.0.1", rover_port=15105, listen_port=15106)
    try:
        client.set_mode(Mode.Teleop)
        client.drive(0.8, -0.4)
        time.sleep(0.05)
        out = core.step(time.monotonic())
        assert out.drive.enabled
        assert abs(out.linear - 0.4) < 1e-6 and abs(out.angular + 0.1) < 1e-6   # scaled

        time.sleep(0.05)
        assert client.poll() >= 1 and client.telemetry().mode == Mode.Teleop

        client.estop()
        time.sleep(0.05)
        out = core.step(time.monotonic())
        assert not out.drive.enabled and out.linear == 0.0 and out.angular == 0.0

        client.release_estop()
        client.drive(0.2, 0.0)
        time.sleep(0.05)
        out = core.step(time.monotonic())
        assert out.drive.enabled and abs(out.linear - 0.1) < 1e-6

        core.local_estop(time.monotonic())                 # rover-side e-stop
        assert core.step(time.monotonic()).drive.reason == StopReason.EStop

        out = core.step(time.monotonic() + 1.0)            # silence -> still stopped, zero
        assert out.linear == 0.0 and not out.drive.enabled
    finally:
        client.close()
        core.close()

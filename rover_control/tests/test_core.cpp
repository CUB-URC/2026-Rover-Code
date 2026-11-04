// C++ tests - no framework needed. Run with `python tools/run_cpp_tests.py` or ctest.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>

#include "rover/command_client.hpp"
#include "rover/controller.hpp"
#include "rover/protocol.hpp"
#include "rover/shaping.hpp"

using namespace rover;

static int failures = 0;
#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
            ++failures;                                                     \
        }                                                                   \
    } while (0)
static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

// ---------------------------------------------------------------- protocol
static void test_roundtrip() {
    auto back = decode(encode(Packet{42, DriveCmd{0.5f, -0.25f}}));
    CHECK(back.has_value() && back->seq == 42);
    auto* d = std::get_if<DriveCmd>(&back->msg);
    CHECK(d && near(d->linear, 0.5f) && near(d->angular, -0.25f));

    auto tb = decode(encode(Packet{7, Telemetry{0.1f, 0.2f, Mode::Autonomous, true, 24.5f}}));
    CHECK(tb && std::get_if<Telemetry>(&tb->msg)->mode == Mode::Autonomous);
}

static void test_corruption_rejected() {
    auto bytes = encode(Packet{1, DriveCmd{1.0f, 0.0f}});
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        auto bad = bytes;
        bad[i] ^= 0x01;
        CHECK(!decode(bad).has_value());
    }
    CHECK(!decode(bytes.data(), bytes.size() - 1).has_value());
}

static void test_stream_decoder() {
    StreamDecoder dec;
    auto a = encode(Packet{1, DriveCmd{0.5f, 0.0f}});
    auto b = encode(Packet{2, EStopCmd{true}});
    std::vector<uint8_t> stream = {0x00, 0xAB, 0x13};      // leading noise
    stream.insert(stream.end(), a.begin(), a.end());
    stream.insert(stream.end(), b.begin(), b.end());
    std::size_t got = 0;
    for (uint8_t byte : stream) got += dec.push(&byte, 1).size();   // slow serial port
    CHECK(got == 2);
    auto bad = encode(Packet{3, DriveCmd{1.0f, 1.0f}});
    bad[9] ^= 0xFF;
    auto good = encode(Packet{4, Heartbeat{}});
    bad.insert(bad.end(), good.begin(), good.end());
    auto pkts = dec.push(bad.data(), bad.size());
    CHECK(pkts.size() == 1 && pkts[0].seq == 4);
    CHECK(dec.corrupt() >= 1);
}

// ---------------------------------------------------------------- shaping
static void test_shaping() {
    CHECK(apply_deadband(0.05f, 0.08f) == 0.0f);
    CHECK(near(apply_deadband(1.0f, 0.08f), 1.0f));
    CHECK(near(apply_deadband(-1.0f, 0.08f), -1.0f));
    CHECK(sanitize(NAN) == 0.0f);
    CHECK(sanitize(5.0f) == 1.0f);
    CHECK(near(shape_axis(1.0f, 0.1f, 0.5f, 0.5f), 0.5f));
    CHECK(shape_axis(NAN, 0.1f, 0.3f) == 0.0f);
}

// ---------------------------------------------------------------- controller
static void test_controller_teleop_and_watchdog() {
    RoverController c;
    CHECK(!c.update(0.0).enabled);                       // idle at start
    c.handle_packet({1, SetModeCmd{Mode::Teleop}}, 0.0);
    c.handle_packet({2, DriveCmd{0.8f, 0.3f}}, 0.0);
    auto out = c.update(0.02);
    CHECK(out.enabled && near(out.linear, 0.8f) && near(out.angular, 0.3f));
    CHECK(out.led == LedState::Blue);
    out = c.update(0.6);                                 // no packets for 0.6 s
    CHECK(!out.enabled && out.reason == StopReason::LinkLost && out.linear == 0.0f);
    c.handle_packet({3, DriveCmd{1.0f, 0.0f}}, 0.61);    // link recovers
    CHECK(c.update(0.62).enabled);
}

static void test_controller_clamps_bad_input() {
    RoverController c;
    c.handle_packet({1, SetModeCmd{Mode::Teleop}}, 0.0);
    c.handle_packet({2, DriveCmd{7.0f, NAN}}, 0.0);
    auto out = c.update(0.01);
    CHECK(out.linear == 1.0f && out.angular == 0.0f);
}

static void test_estop_latches() {
    RoverController c;
    c.handle_packet({1, SetModeCmd{Mode::Teleop}}, 0.0);
    c.handle_packet({2, EStopCmd{true}}, 0.0);
    c.handle_packet({3, DriveCmd{1.0f, 0.0f}}, 0.01);
    auto out = c.update(0.02);
    CHECK(!out.enabled && out.reason == StopReason::EStop);
    c.handle_packet({4, EStopCmd{false}}, 0.03);
    out = c.update(0.04);
    CHECK(out.enabled && out.linear == 0.0f);            // command reset on release
    c.handle_packet({1, EStopCmd{true}}, 0.05);          // stale e-stop still obeyed
    CHECK(c.estopped());
}

static void test_stale_and_wrap() {
    RoverController c;
    CHECK(c.handle_packet({65534, Heartbeat{}}, 0.0));
    CHECK(c.handle_packet({1, Heartbeat{}}, 0.01));      // wrapped: still newer
    CHECK(!c.handle_packet({65535, Heartbeat{}}, 0.02)); // older: rejected
    CHECK(c.stats().stale == 1);
}

static void test_autonomy_ignores_teleop() {
    RoverController c;
    c.handle_packet({1, SetModeCmd{Mode::Autonomous}}, 0.0);
    c.handle_packet({2, DriveCmd{1.0f, 0.0f}}, 0.0);
    auto out = c.update(0.01);
    CHECK(!out.enabled && out.reason == StopReason::NoAutonomyCmd && out.led == LedState::Red);
    c.set_autonomy_command(-0.5f, 0.2f, 0.02);
    out = c.update(0.03);
    CHECK(out.enabled && near(out.linear, -0.5f) && near(out.angular, 0.2f));
}

// ---------------------------------------------------------------- command client
// A fake radio: client -> rover packets go into `uplink`, rover -> client into `downlink`.
struct FakeRadio {
    std::deque<std::vector<uint8_t>> uplink, downlink;
    CommandClient::SendFn send() { return [this](const std::vector<uint8_t>& b) { uplink.push_back(b); }; }
    CommandClient::RecvFn recv() {
        return [this]() {
            if (downlink.empty()) return std::vector<uint8_t>{};
            auto b = downlink.front();
            downlink.pop_front();
            return b;
        };
    }
};

static void test_client_commands() {
    FakeRadio radio;
    CommandClient client(radio.send(), radio.recv());

    client.drive(0.5f, -0.25f);
    CHECK(radio.uplink.size() == 1);
    auto pkt = decode(radio.uplink.back());
    CHECK(pkt && pkt->seq == 1);
    auto* d = std::get_if<DriveCmd>(&pkt->msg);
    CHECK(d && near(d->linear, 0.5f) && near(d->angular, -0.25f));

    client.drive(NAN, 9.0f);                              // sanitized before sending
    d = std::get_if<DriveCmd>(&decode(radio.uplink.back())->msg);
    CHECK(d && d->linear == 0.0f && d->angular == 1.0f);

    radio.uplink.clear();
    client.estop();                                       // critical: sent 3x
    CHECK(radio.uplink.size() == 3);
    for (auto& b : radio.uplink) CHECK(std::get_if<EStopCmd>(&decode(b)->msg)->engage);
    CHECK(client.stats().packets_sent == 5);
    CHECK(client.last_seq() == 5);
}

static void test_client_telemetry_and_link() {
    FakeRadio radio;
    CommandClient client(radio.send(), radio.recv());
    CHECK(!client.link_ok(0.0) && !client.telemetry().has_value());

    radio.downlink.push_back(encode(Packet{1, Telemetry{0.3f, 0.0f, Mode::Teleop, false, 25.0f}}));
    radio.downlink.push_back({0xAB, 0xCD, 0x00});        // junk is ignored
    CHECK(client.poll(10.0) == 1);
    CHECK(client.telemetry()->mode == Mode::Teleop);
    CHECK(client.link_ok(10.5));
    CHECK(!client.link_ok(11.5));                        // older than 1 s
}

static void test_client_to_controller_end_to_end() {
    // Client and RoverController wired back to back, as over a real link.
    FakeRadio radio;
    CommandClient client(radio.send(), radio.recv());
    RoverController rover;
    auto deliver = [&](double t) {
        for (auto& b : radio.uplink) rover.handle_bytes(b.data(), b.size(), t);
        radio.uplink.clear();
    };
    client.set_mode(Mode::Teleop);
    client.drive(0.6f, 0.1f);
    deliver(0.0);
    auto out = rover.update(0.01);
    CHECK(out.enabled && near(out.linear, 0.6f) && near(out.angular, 0.1f));

    client.estop();
    deliver(0.02);
    CHECK(rover.update(0.03).reason == StopReason::EStop);
    client.release_estop();
    client.drive(0.2f, 0.0f);
    deliver(0.04);
    out = rover.update(0.05);
    CHECK(out.enabled && near(out.linear, 0.2f));

    radio.downlink.push_back(encode(rover.make_telemetry(1, 24.0f)));
    client.poll(0.06);
    CHECK(client.telemetry() && near(client.telemetry()->linear, 0.2f));
}

int main() {
    test_roundtrip();
    test_corruption_rejected();
    test_stream_decoder();
    test_shaping();
    test_controller_teleop_and_watchdog();
    test_controller_clamps_bad_input();
    test_estop_latches();
    test_stale_and_wrap();
    test_autonomy_ignores_teleop();
    test_client_commands();
    test_client_telemetry_and_link();
    test_client_to_controller_end_to_end();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("all C++ tests passed\n");
    return EXIT_SUCCESS;
}

// CommandClient: the base-station side of the command link.
//
// One small class with one function per command. It builds packets, numbers them,
// repeats safety-critical ones, and tracks what the rover reports back. It does
// not open sockets or serial ports itself: you hand it two functions, one that
// sends bytes and one that returns received bytes. That keeps this code
// portable (Windows, Linux, Jetson) and lets the same class drive a UDP link,
// an RFD900 serial modem, or a fake link in a unit test.
//
//   CommandClient rover(send_fn, recv_fn);
//   rover.set_mode(Mode::Teleop);
//   rover.drive(0.5f, 0.0f);   // call ~20x per second while driving
//   rover.poll();              // read telemetry
//   rover.estop();
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "rover/protocol.hpp"

namespace rover {

struct ClientConfig {
    int critical_repeats = 3;          // e-stop / mode messages are sent this many times
    double telemetry_timeout_s = 1.0;  // no telemetry for this long => link_ok() is false
};

struct ClientStats {
    uint32_t packets_sent = 0;
    uint32_t telemetry_received = 0;
    uint32_t corrupt = 0;              // bytes that failed CRC/length checks
};

class CommandClient {
public:
    using SendFn = std::function<void(const std::vector<uint8_t>&)>;
    using RecvFn = std::function<std::vector<uint8_t>()>;   // empty vector = nothing waiting

    CommandClient(SendFn send, RecvFn recv, ClientConfig cfg = {});

    // ---- commands -------------------------------------------------------------
    // linear: -1 (reverse) .. +1 (forward). angular: -1 (right) .. +1 (left), ROS convention.
    // Values are clamped to [-1, 1]; NaN becomes 0.
    void drive(float linear, float angular);
    void stop() { drive(0.0f, 0.0f); }
    void estop();                     // latches on the rover until release_estop()
    void release_estop();
    void set_mode(Mode mode);
    void heartbeat();                 // keep the link alive without driving

    // ---- feedback -------------------------------------------------------------
    // Reads everything waiting on the link. Call once per loop. Returns the number
    // of telemetry packets received. now_s < 0 means "use the steady clock".
    int poll(double now_s = -1.0);
    // Feed bytes directly (if you read the link yourself).
    int receive(const uint8_t* data, std::size_t len, double now_s = -1.0);

    std::optional<Telemetry> telemetry() const { return last_telemetry_; }
    bool link_ok(double now_s = -1.0) const;
    double telemetry_age(double now_s = -1.0) const;   // seconds; -1 if never received
    const ClientStats& stats() const { return stats_; }
    uint16_t last_seq() const { return seq_; }

    static double now();   // steady clock, seconds

private:
    void send_msg(const Message& msg, int repeats = 1);

    SendFn send_;
    RecvFn recv_;
    ClientConfig cfg_;
    StreamDecoder decoder_;
    uint16_t seq_ = 0;
    std::optional<Telemetry> last_telemetry_;
    double last_rx_s_ = -1.0;
    ClientStats stats_;
};

}  // namespace rover

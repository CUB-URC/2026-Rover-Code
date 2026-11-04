#include "rover/command_client.hpp"

#include <chrono>
#include <utility>

#include "rover/shaping.hpp"

namespace rover {

CommandClient::CommandClient(SendFn send, RecvFn recv, ClientConfig cfg)
    : send_(std::move(send)), recv_(std::move(recv)), cfg_(cfg) {}

double CommandClient::now() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

void CommandClient::send_msg(const Message& msg, int repeats) {
    if (!send_) return;
    for (int i = 0; i < repeats; ++i) {
        // Each copy gets its own sequence number; the rover ignores duplicates of
        // state it already has, so repeats are harmless and survive packet loss.
        send_(encode(Packet{++seq_, msg}));
        ++stats_.packets_sent;
    }
}

void CommandClient::drive(float linear, float angular) {
    send_msg(DriveCmd{sanitize(linear), sanitize(angular)});
}

void CommandClient::estop() { send_msg(EStopCmd{true}, cfg_.critical_repeats); }

void CommandClient::release_estop() { send_msg(EStopCmd{false}, cfg_.critical_repeats); }

void CommandClient::set_mode(Mode mode) { send_msg(SetModeCmd{mode}, cfg_.critical_repeats); }

void CommandClient::heartbeat() { send_msg(Heartbeat{}); }

int CommandClient::receive(const uint8_t* data, std::size_t len, double now_s) {
    if (now_s < 0) now_s = now();
    const uint32_t corrupt_before = decoder_.corrupt();
    int got = 0;
    // StreamDecoder handles both whole UDP datagrams and split serial reads.
    for (const Packet& pkt : decoder_.push(data, len)) {
        if (auto* t = std::get_if<Telemetry>(&pkt.msg)) {
            last_telemetry_ = *t;
            last_rx_s_ = now_s;
            ++stats_.telemetry_received;
            ++got;
        }
    }
    stats_.corrupt += decoder_.corrupt() - corrupt_before;
    return got;
}

int CommandClient::poll(double now_s) {
    if (!recv_) return 0;
    int got = 0;
    for (int i = 0; i < 64; ++i) {          // bounded, so a flood can't stall the caller
        std::vector<uint8_t> chunk = recv_();
        if (chunk.empty()) break;
        got += receive(chunk.data(), chunk.size(), now_s);
    }
    return got;
}

double CommandClient::telemetry_age(double now_s) const {
    if (last_rx_s_ < 0) return -1.0;
    if (now_s < 0) now_s = now();
    return now_s - last_rx_s_;
}

bool CommandClient::link_ok(double now_s) const {
    const double age = telemetry_age(now_s);
    return age >= 0 && age <= cfg_.telemetry_timeout_s;
}

}  // namespace rover

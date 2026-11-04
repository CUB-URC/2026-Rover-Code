#include "rover/controller.hpp"

#include "rover/shaping.hpp"

namespace rover {

LedState led_for_mode(Mode mode) {
    switch (mode) {
        case Mode::Teleop: return LedState::Blue;
        case Mode::Autonomous: return LedState::Red;
        case Mode::Arrived: return LedState::FlashingGreen;
        default: return LedState::Off;
    }
}

RoverController::RoverController(ControllerConfig cfg)
    : cfg_(cfg), link_wd_(cfg.watchdog_timeout_s), autonomy_wd_(cfg.autonomy_timeout_s) {}

bool RoverController::is_fresh(uint16_t seq, double now_s) const {
    // After a link loss we accept any sequence so a restarted base station can reconnect.
    if (!have_seq_ || link_wd_.expired(now_s)) return true;
    // Signed difference handles 16-bit wrap-around (65535 -> 0).
    const int16_t diff = static_cast<int16_t>(static_cast<uint16_t>(seq - last_seq_));
    return diff > 0;
}

bool RoverController::handle_bytes(const uint8_t* data, std::size_t len, double now_s) {
    auto pkt = decode(data, len);
    if (!pkt) {
        ++stats_.corrupt;
        return false;
    }
    return handle_packet(*pkt, now_s);
}

bool RoverController::handle_packet(const Packet& pkt, double now_s) {
    // E-stop engage is honored even if it looks stale: never drop a stop request.
    if (auto* e = std::get_if<EStopCmd>(&pkt.msg); e && e->engage) {
        estop_ = true;
    }
    if (!is_fresh(pkt.seq, now_s)) {
        ++stats_.stale;
        return false;
    }
    ++stats_.accepted;
    have_seq_ = true;
    last_seq_ = pkt.seq;
    link_wd_.feed(now_s);

    if (auto* d = std::get_if<DriveCmd>(&pkt.msg)) {
        teleop_cmd_ = {sanitize(d->linear), sanitize(d->angular)};
    } else if (auto* e = std::get_if<EStopCmd>(&pkt.msg)) {
        if (!e->engage) {
            estop_ = false;
            teleop_cmd_ = {};   // require a fresh command after release
        }
    } else if (auto* m = std::get_if<SetModeCmd>(&pkt.msg)) {
        if (m->mode != mode_) {
            mode_ = m->mode;
            teleop_cmd_ = {};
            autonomy_cmd_ = {};
        }
    }
    // Heartbeat: only feeds the watchdog. Telemetry: not expected on the rover; ignored.
    return true;
}

void RoverController::set_autonomy_command(float linear, float angular, double now_s) {
    autonomy_cmd_ = {sanitize(linear), sanitize(angular)};
    autonomy_wd_.feed(now_s);
}

DriveOutput RoverController::update(double now_s) {
    DriveOutput out;
    out.led = led_for_mode(mode_);

    StopReason reason = StopReason::None;
    DriveCmd cmd{};
    const bool link_lost = link_wd_.expired(now_s);

    if (estop_) {
        reason = StopReason::EStop;
    } else if (mode_ == Mode::Teleop) {
        if (link_lost) reason = StopReason::LinkLost;
        else cmd = teleop_cmd_;
    } else if (mode_ == Mode::Autonomous) {
        if (link_lost && cfg_.stop_on_link_loss_in_autonomy) reason = StopReason::LinkLost;
        else if (autonomy_wd_.expired(now_s)) reason = StopReason::NoAutonomyCmd;
        else cmd = autonomy_cmd_;
    } else {
        reason = StopReason::Idle;   // Idle and Arrived: hold still
    }

    if (reason == StopReason::None) {
        out.linear = cmd.linear;
        out.angular = cmd.angular;
        out.enabled = true;
    }
    out.reason = reason;
    last_out_ = out;
    return out;
}

Packet RoverController::make_telemetry(uint16_t seq, float battery_v) const {
    return Packet{seq, Telemetry{last_out_.linear, last_out_.angular, mode_, estop_, battery_v}};
}

}  // namespace rover

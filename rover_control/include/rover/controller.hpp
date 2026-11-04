// RoverController: rover-side safety gate for incoming commands.
//
// Feed it raw bytes from the radio (handle_bytes) and, in autonomous mode,
// commands from the autonomy stack (set_autonomy_command). Call update() at a
// fixed rate (e.g. 50 Hz). The returned DriveOutput is what should be published
// to /drive/cmd_vel (linear.x, angular.z) for the drive team's drive_controller.
//
// Safety rules enforced here:
//   * E-stop latches: once engaged, output stays at zero until an explicit release.
//   * Link loss: if no valid packet arrives within watchdog_timeout_s, output is zero.
//   * Stale/reordered UDP packets (older sequence numbers) are ignored.
//   * In Autonomous mode, drive commands from the base station are ignored
//     (URC does not allow teleoperation during autonomous legs).
//
// Wheel mixing, acceleration limits and speed caps are NOT done here: the drive
// team's drive_controller owns those.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rover/protocol.hpp"
#include "rover/safety.hpp"

namespace rover {

// URC autonomy indicator states. Check the current year's rules for exact colors;
// historically: red = autonomous, blue = teleoperation, flashing green = arrived.
enum class LedState : uint8_t { Off = 0, Red = 1, Blue = 2, FlashingGreen = 3 };

LedState led_for_mode(Mode mode);

struct ControllerConfig {
    double watchdog_timeout_s = 0.5;      // base-station link
    double autonomy_timeout_s = 0.3;      // autonomy stack must keep publishing
    bool stop_on_link_loss_in_autonomy = true;
};

enum class StopReason : uint8_t { None = 0, EStop = 1, LinkLost = 2, Idle = 3, NoAutonomyCmd = 4 };

// What the rover should do right now. When enabled is false, linear/angular are 0
// and the drive system should hold still.
struct DriveOutput {
    float linear = 0.0f;     // -> geometry_msgs/Twist linear.x
    float angular = 0.0f;    // -> geometry_msgs/Twist angular.z (+ = turn left)
    bool enabled = false;
    StopReason reason = StopReason::Idle;
    LedState led = LedState::Off;
};

struct LinkStats {
    uint32_t accepted = 0;
    uint32_t corrupt = 0;   // failed decode (bad CRC, length, etc.)
    uint32_t stale = 0;     // older sequence number than one already seen
};

class RoverController {
public:
    explicit RoverController(ControllerConfig cfg = {});

    // Returns true if the bytes decoded into a valid, fresh packet.
    bool handle_bytes(const uint8_t* data, std::size_t len, double now_s);
    // Returns true if the packet was fresh and applied.
    bool handle_packet(const Packet& pkt, double now_s);

    // Commands from the onboard autonomy stack (only used in Autonomous mode).
    void set_autonomy_command(float linear, float angular, double now_s);

    DriveOutput update(double now_s);

    // Builds a telemetry packet for the base station.
    Packet make_telemetry(uint16_t seq, float battery_v) const;

    bool estopped() const { return estop_; }
    Mode mode() const { return mode_; }
    bool link_ok(double now_s) const { return !link_wd_.expired(now_s); }
    const LinkStats& stats() const { return stats_; }
    const ControllerConfig& config() const { return cfg_; }

private:
    bool is_fresh(uint16_t seq, double now_s) const;

    ControllerConfig cfg_;
    Watchdog link_wd_;
    Watchdog autonomy_wd_;

    Mode mode_ = Mode::Idle;
    bool estop_ = false;
    DriveCmd teleop_cmd_{};
    DriveCmd autonomy_cmd_{};
    bool have_seq_ = false;
    uint16_t last_seq_ = 0;
    DriveOutput last_out_{};
    LinkStats stats_{};
};

}  // namespace rover

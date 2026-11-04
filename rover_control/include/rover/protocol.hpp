// Wire protocol between base station and rover.
//
// Frame layout (little-endian):
//   [0xAB][0xCD][version][type][seq_lo][seq_hi][len][payload ... len bytes][crc_lo][crc_hi]
// CRC-16/CCITT-FALSE covers everything from `version` through the payload.
// Frames are small (< 32 bytes) so they fit in a single UDP datagram or serial burst.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

namespace rover {

constexpr uint8_t kMagic0 = 0xAB;
constexpr uint8_t kMagic1 = 0xCD;
// v2: DriveCmd/Telemetry use ROS conventions (linear/angular, +angular = turn left).
constexpr uint8_t kProtocolVersion = 2;
constexpr std::size_t kHeaderSize = 7;   // magic(2) + version + type + seq(2) + len
constexpr std::size_t kCrcSize = 2;

enum class MsgType : uint8_t {
    Heartbeat = 1,
    Drive = 2,
    EStop = 3,
    SetMode = 4,
    Telemetry = 5,
};

// Operating mode. Drives the URC autonomy indicator (see LedState in controller.hpp).
enum class Mode : uint8_t {
    Idle = 0,
    Teleop = 1,
    Autonomous = 2,
    Arrived = 3,   // autonomous leg reached its target
};

struct Heartbeat {};

// Follows ROS conventions (REP-103) so it maps 1:1 onto geometry_msgs/Twist on
// /drive/cmd_vel:  linear -> linear.x,  angular -> angular.z.
struct DriveCmd {
    float linear = 0.0f;    // -1 (full reverse) .. +1 (full forward)
    float angular = 0.0f;   // -1 (turn right)   .. +1 (turn LEFT)   <- ROS sign convention
};

struct EStopCmd {
    bool engage = true;     // true = stop everything, false = release
};

struct SetModeCmd {
    Mode mode = Mode::Idle;
};

struct Telemetry {
    float linear = 0.0f;     // drive command currently passed to the drive system
    float angular = 0.0f;
    Mode mode = Mode::Idle;
    bool estop = false;
    float battery_v = 0.0f;
};

using Message = std::variant<Heartbeat, DriveCmd, EStopCmd, SetModeCmd, Telemetry>;

struct Packet {
    uint16_t seq = 0;
    Message msg = Heartbeat{};
};

uint16_t crc16_ccitt(const uint8_t* data, std::size_t len);

std::vector<uint8_t> encode(const Packet& pkt);

// Returns std::nullopt for anything malformed: bad magic, version, length, CRC or type.
std::optional<Packet> decode(const uint8_t* data, std::size_t len);
inline std::optional<Packet> decode(const std::vector<uint8_t>& buf) {
    return decode(buf.data(), buf.size());
}

MsgType type_of(const Message& msg);

// Byte-stream decoder for serial links (e.g. RFD900 900 MHz modems), where frames
// arrive split across reads or with noise between them. Push whatever bytes you
// read; complete, valid frames come out. Resyncs on the magic bytes after errors.
class StreamDecoder {
public:
    static constexpr std::size_t kMaxPayload = 64;
    static constexpr std::size_t kMaxBuffer = 1024;

    std::vector<Packet> push(const uint8_t* data, std::size_t len);
    uint32_t corrupt() const { return corrupt_; }
    void reset() { buf_.clear(); }

private:
    std::vector<uint8_t> buf_;
    uint32_t corrupt_ = 0;
};

}  // namespace rover

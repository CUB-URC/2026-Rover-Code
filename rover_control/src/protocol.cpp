#include "rover/protocol.hpp"

#include <cstring>
#include <type_traits>

namespace rover {
namespace {

// Floats are sent as their IEEE-754 bit pattern, little-endian, so the format
// does not depend on the host's byte order.
void put_u16(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>(v >> 8));
}
void put_f32(std::vector<uint8_t>& out, float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof bits);
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(bits >> (8 * i)));
}
float get_f32(const uint8_t* p) {
    uint32_t bits = 0;
    for (int i = 0; i < 4; ++i) bits |= static_cast<uint32_t>(p[i]) << (8 * i);
    float f;
    std::memcpy(&f, &bits, sizeof f);
    return f;
}

std::vector<uint8_t> encode_payload(const Message& msg) {
    std::vector<uint8_t> p;
    std::visit(
        [&p](const auto& m) {
            using T = std::decay_t<decltype(m)>;
            if constexpr (std::is_same_v<T, DriveCmd>) {
                put_f32(p, m.linear);
                put_f32(p, m.angular);
            } else if constexpr (std::is_same_v<T, EStopCmd>) {
                p.push_back(m.engage ? 1 : 0);
            } else if constexpr (std::is_same_v<T, SetModeCmd>) {
                p.push_back(static_cast<uint8_t>(m.mode));
            } else if constexpr (std::is_same_v<T, Telemetry>) {
                put_f32(p, m.linear);
                put_f32(p, m.angular);
                p.push_back(static_cast<uint8_t>(m.mode));
                p.push_back(m.estop ? 1 : 0);
                put_f32(p, m.battery_v);
            }
            // Heartbeat: empty payload
        },
        msg);
    return p;
}

bool valid_mode(uint8_t v) { return v <= static_cast<uint8_t>(Mode::Arrived); }

}  // namespace

uint16_t crc16_ccitt(const uint8_t* data, std::size_t len) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

MsgType type_of(const Message& msg) {
    switch (msg.index()) {
        case 0: return MsgType::Heartbeat;
        case 1: return MsgType::Drive;
        case 2: return MsgType::EStop;
        case 3: return MsgType::SetMode;
        default: return MsgType::Telemetry;
    }
}

std::vector<uint8_t> encode(const Packet& pkt) {
    const auto payload = encode_payload(pkt.msg);
    std::vector<uint8_t> out;
    out.reserve(kHeaderSize + payload.size() + kCrcSize);
    out.push_back(kMagic0);
    out.push_back(kMagic1);
    out.push_back(kProtocolVersion);
    out.push_back(static_cast<uint8_t>(type_of(pkt.msg)));
    put_u16(out, pkt.seq);
    out.push_back(static_cast<uint8_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());
    put_u16(out, crc16_ccitt(out.data() + 2, out.size() - 2));
    return out;
}

std::optional<Packet> decode(const uint8_t* data, std::size_t len) {
    if (data == nullptr || len < kHeaderSize + kCrcSize) return std::nullopt;
    if (data[0] != kMagic0 || data[1] != kMagic1) return std::nullopt;
    if (data[2] != kProtocolVersion) return std::nullopt;

    const uint8_t type = data[3];
    const uint16_t seq = static_cast<uint16_t>(data[4] | (data[5] << 8));
    const std::size_t plen = data[6];
    if (len != kHeaderSize + plen + kCrcSize) return std::nullopt;

    const uint16_t got_crc =
        static_cast<uint16_t>(data[kHeaderSize + plen] | (data[kHeaderSize + plen + 1] << 8));
    if (crc16_ccitt(data + 2, kHeaderSize - 2 + plen) != got_crc) return std::nullopt;

    const uint8_t* p = data + kHeaderSize;
    Packet pkt;
    pkt.seq = seq;
    switch (static_cast<MsgType>(type)) {
        case MsgType::Heartbeat:
            if (plen != 0) return std::nullopt;
            pkt.msg = Heartbeat{};
            break;
        case MsgType::Drive:
            if (plen != 8) return std::nullopt;
            pkt.msg = DriveCmd{get_f32(p), get_f32(p + 4)};
            break;
        case MsgType::EStop:
            if (plen != 1) return std::nullopt;
            pkt.msg = EStopCmd{p[0] != 0};
            break;
        case MsgType::SetMode:
            if (plen != 1 || !valid_mode(p[0])) return std::nullopt;
            pkt.msg = SetModeCmd{static_cast<Mode>(p[0])};
            break;
        case MsgType::Telemetry:
            if (plen != 14 || !valid_mode(p[8])) return std::nullopt;
            pkt.msg = Telemetry{get_f32(p), get_f32(p + 4), static_cast<Mode>(p[8]), p[9] != 0,
                                get_f32(p + 10)};
            break;
        default:
            return std::nullopt;
    }
    return pkt;
}

std::vector<Packet> StreamDecoder::push(const uint8_t* data, std::size_t len) {
    std::vector<Packet> out;
    if (data && len) buf_.insert(buf_.end(), data, data + len);
    if (buf_.size() > kMaxBuffer) {          // garbage flood: keep only the tail
        buf_.erase(buf_.begin(), buf_.end() - static_cast<std::ptrdiff_t>(kMaxBuffer / 2));
    }

    std::size_t pos = 0;
    while (true) {
        // Find the next magic sequence.
        while (pos + 1 < buf_.size() && !(buf_[pos] == kMagic0 && buf_[pos + 1] == kMagic1)) ++pos;
        if (pos + kHeaderSize > buf_.size()) break;          // need more bytes

        const std::size_t plen = buf_[pos + 6];
        if (plen > kMaxPayload) { ++corrupt_; ++pos; continue; }
        const std::size_t total = kHeaderSize + plen + kCrcSize;
        if (pos + total > buf_.size()) break;                // frame not complete yet

        if (auto pkt = decode(buf_.data() + pos, total)) {
            out.push_back(*pkt);
            pos += total;
        } else {
            ++corrupt_;
            ++pos;                                            // resync one byte later
        }
    }
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos));
    return out;
}

}  // namespace rover

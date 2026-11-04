// pybind11 bindings: exposes the C++ core as the Python module `rover._core`.
// Python code should import from `rover` (see python/rover/__init__.py).
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>   // std::optional / std::variant <-> Python

#include "rover/command_client.hpp"
#include "rover/controller.hpp"
#include "rover/protocol.hpp"
#include "rover/safety.hpp"
#include "rover/shaping.hpp"

namespace py = pybind11;
using namespace rover;

namespace {
std::string to_str(const py::bytes& b) { return static_cast<std::string>(b); }
py::bytes to_bytes(const std::vector<uint8_t>& v) {
    return py::bytes(reinterpret_cast<const char*>(v.data()), v.size());
}
const uint8_t* as_u8(const std::string& s) { return reinterpret_cast<const uint8_t*>(s.data()); }
}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "Rover control core (C++17) - protocol, drive mixing, safety";

    // ---- enums -------------------------------------------------------------
    py::enum_<MsgType>(m, "MsgType")
        .value("Heartbeat", MsgType::Heartbeat)
        .value("Drive", MsgType::Drive)
        .value("EStop", MsgType::EStop)
        .value("SetMode", MsgType::SetMode)
        .value("Telemetry", MsgType::Telemetry);

    py::enum_<Mode>(m, "Mode")
        .value("Idle", Mode::Idle)
        .value("Teleop", Mode::Teleop)
        .value("Autonomous", Mode::Autonomous)
        .value("Arrived", Mode::Arrived);

    py::enum_<LedState>(m, "LedState")
        .value("Off", LedState::Off)
        .value("Red", LedState::Red)
        .value("Blue", LedState::Blue)
        .value("FlashingGreen", LedState::FlashingGreen);

    py::enum_<StopReason>(m, "StopReason")
        .value("None_", StopReason::None)
        .value("EStop", StopReason::EStop)
        .value("LinkLost", StopReason::LinkLost)
        .value("Idle", StopReason::Idle)
        .value("NoAutonomyCmd", StopReason::NoAutonomyCmd);

    // ---- messages ------------------------------------------------------------
    py::class_<Heartbeat>(m, "Heartbeat")
        .def(py::init<>())
        .def("__repr__", [](const Heartbeat&) { return "Heartbeat()"; });

    py::class_<DriveCmd>(m, "DriveCmd")
        .def(py::init<float, float>(), py::arg("linear") = 0.0f, py::arg("angular") = 0.0f)
        .def_readwrite("linear", &DriveCmd::linear)
        .def_readwrite("angular", &DriveCmd::angular)
        .def("__repr__", [](const DriveCmd& d) {
            return "DriveCmd(linear=" + std::to_string(d.linear) +
                   ", angular=" + std::to_string(d.angular) + ")";
        });

    py::class_<EStopCmd>(m, "EStopCmd")
        .def(py::init<bool>(), py::arg("engage") = true)
        .def_readwrite("engage", &EStopCmd::engage)
        .def("__repr__", [](const EStopCmd& e) {
            return std::string("EStopCmd(engage=") + (e.engage ? "True" : "False") + ")";
        });

    py::class_<SetModeCmd>(m, "SetModeCmd")
        .def(py::init<Mode>(), py::arg("mode"))
        .def_readwrite("mode", &SetModeCmd::mode)
        .def("__repr__", [](const SetModeCmd& c) {
            static const char* names[] = {"Idle", "Teleop", "Autonomous", "Arrived"};
            return std::string("SetModeCmd(Mode.") + names[static_cast<int>(c.mode)] + ")";
        });

    py::class_<Telemetry>(m, "Telemetry")
        .def(py::init<>())
        .def_readwrite("linear", &Telemetry::linear)
        .def_readwrite("angular", &Telemetry::angular)
        .def_readwrite("mode", &Telemetry::mode)
        .def_readwrite("estop", &Telemetry::estop)
        .def_readwrite("battery_v", &Telemetry::battery_v);

    py::class_<Packet>(m, "Packet")
        .def(py::init<uint16_t, Message>(), py::arg("seq"), py::arg("msg"))
        .def_readwrite("seq", &Packet::seq)
        .def_readwrite("msg", &Packet::msg)
        .def_property_readonly("type", [](const Packet& p) { return type_of(p.msg); });

    m.def("encode", [](const Packet& p) {
        auto v = encode(p);
        return py::bytes(reinterpret_cast<const char*>(v.data()), v.size());
    }, py::arg("packet"), "Serialize a Packet to bytes ready to send.");

    m.def("decode", [](const py::bytes& data) {
        const std::string s = to_str(data);
        return decode(as_u8(s), s.size());
    }, py::arg("data"), "Parse bytes into a Packet, or None if invalid.");

    m.def("crc16_ccitt", [](const py::bytes& data) {
        const std::string s = to_str(data);
        return crc16_ccitt(as_u8(s), s.size());
    });

    py::class_<StreamDecoder>(m, "StreamDecoder")
        .def(py::init<>())
        .def("push", [](StreamDecoder& d, const py::bytes& data) {
            const std::string s = to_str(data);
            return d.push(as_u8(s), s.size());
        }, py::arg("data"), "Feed raw serial bytes; returns a list of complete Packets.")
        .def("reset", &StreamDecoder::reset)
        .def_property_readonly("corrupt", &StreamDecoder::corrupt);

    // ---- input shaping ---------------------------------------------------------
    m.def("sanitize", &sanitize, py::arg("x"), "Clamp to [-1, 1]; NaN becomes 0.");
    m.def("apply_deadband", &apply_deadband, py::arg("x"), py::arg("deadband"));
    m.def("apply_expo", &apply_expo, py::arg("x"), py::arg("expo"));
    m.def("shape_axis", &shape_axis, py::arg("x"), py::arg("deadband"), py::arg("expo"),
          py::arg("scale") = 1.0f, "sanitize -> deadband -> expo -> scale for one stick axis.");

    py::class_<Watchdog>(m, "Watchdog")
        .def(py::init<double>(), py::arg("timeout_s"))
        .def("feed", &Watchdog::feed, py::arg("now_s"))
        .def("expired", &Watchdog::expired, py::arg("now_s"));

    // ---- controller --------------------------------------------------------------
    py::class_<ControllerConfig>(m, "ControllerConfig")
        .def(py::init<>())
        .def_readwrite("watchdog_timeout_s", &ControllerConfig::watchdog_timeout_s)
        .def_readwrite("autonomy_timeout_s", &ControllerConfig::autonomy_timeout_s)
        .def_readwrite("stop_on_link_loss_in_autonomy",
                       &ControllerConfig::stop_on_link_loss_in_autonomy);

    py::class_<DriveOutput>(m, "DriveOutput")
        .def_readonly("linear", &DriveOutput::linear)
        .def_readonly("angular", &DriveOutput::angular)
        .def_readonly("enabled", &DriveOutput::enabled)
        .def_readonly("reason", &DriveOutput::reason)
        .def_readonly("led", &DriveOutput::led);

    py::class_<LinkStats>(m, "LinkStats")
        .def_readonly("accepted", &LinkStats::accepted)
        .def_readonly("corrupt", &LinkStats::corrupt)
        .def_readonly("stale", &LinkStats::stale);

    py::class_<RoverController>(m, "RoverController")
        .def(py::init<ControllerConfig>(), py::arg("cfg") = ControllerConfig{})
        .def("handle_bytes", [](RoverController& c, const py::bytes& data, double now) {
            const std::string s = to_str(data);
            return c.handle_bytes(as_u8(s), s.size(), now);
        }, py::arg("data"), py::arg("now_s"))
        .def("handle_packet", &RoverController::handle_packet, py::arg("packet"), py::arg("now_s"))
        .def("set_autonomy_command", &RoverController::set_autonomy_command,
             py::arg("linear"), py::arg("angular"), py::arg("now_s"))
        .def("update", &RoverController::update, py::arg("now_s"))
        .def("make_telemetry", &RoverController::make_telemetry, py::arg("seq"),
             py::arg("battery_v") = 0.0f)
        .def("link_ok", &RoverController::link_ok, py::arg("now_s"))
        .def_property_readonly("estopped", &RoverController::estopped)
        .def_property_readonly("mode", &RoverController::mode)
        .def_property_readonly("stats", &RoverController::stats)
        .def_property_readonly("config", &RoverController::config);

    m.def("led_for_mode", &led_for_mode, py::arg("mode"));

    // ---- command client (base station) -------------------------------------------
    py::class_<ClientConfig>(m, "ClientConfig")
        .def(py::init<>())
        .def_readwrite("critical_repeats", &ClientConfig::critical_repeats)
        .def_readwrite("telemetry_timeout_s", &ClientConfig::telemetry_timeout_s);

    py::class_<ClientStats>(m, "ClientStats")
        .def_readonly("packets_sent", &ClientStats::packets_sent)
        .def_readonly("telemetry_received", &ClientStats::telemetry_received)
        .def_readonly("corrupt", &ClientStats::corrupt);

    // py::dynamic_attr lets Python attach the link object (client.link = ...).
    py::class_<CommandClient>(m, "CommandClient", py::dynamic_attr())
        .def(py::init([](py::function send, py::function recv, ClientConfig cfg) {
                 // Wrap the Python callables as C++ std::functions. pybind11 holds a
                 // reference to each, and we take the GIL whenever C++ calls them.
                 CommandClient::SendFn send_fn = [send](const std::vector<uint8_t>& b) {
                     py::gil_scoped_acquire gil;
                     send(to_bytes(b));
                 };
                 CommandClient::RecvFn recv_fn = [recv]() {
                     py::gil_scoped_acquire gil;
                     py::object r = recv();
                     if (r.is_none()) return std::vector<uint8_t>{};
                     const std::string s = r.cast<py::bytes>();
                     return std::vector<uint8_t>(s.begin(), s.end());
                 };
                 return CommandClient(send_fn, recv_fn, cfg);
             }),
             py::arg("send"), py::arg("recv"), py::arg("cfg") = ClientConfig{},
             "send(bytes) transmits one packet; recv() returns waiting bytes or b''/None.")
        .def("drive", &CommandClient::drive, py::arg("linear"), py::arg("angular") = 0.0f,
             "linear -1..1 (forward +), angular -1..1 (LEFT +, ROS convention).")
        .def("stop", &CommandClient::stop, "Send a zero drive command.")
        .def("estop", &CommandClient::estop, "Engage the e-stop (latches; sent 3x).")
        .def("release_estop", &CommandClient::release_estop, "Release a latched e-stop (sent 3x).")
        .def("set_mode", &CommandClient::set_mode, py::arg("mode"), "Change rover mode (sent 3x).")
        .def("heartbeat", &CommandClient::heartbeat, "Keep the link alive without driving.")
        .def("poll", &CommandClient::poll, py::arg("now_s") = -1.0,
             "Read all waiting telemetry. Returns how many telemetry packets arrived.")
        .def("receive", [](CommandClient& c, const py::bytes& data, double now_s) {
            const std::string s = to_str(data);
            return c.receive(as_u8(s), s.size(), now_s);
        }, py::arg("data"), py::arg("now_s") = -1.0)
        .def("telemetry", &CommandClient::telemetry, "Latest Telemetry from the rover, or None.")
        .def("link_ok", &CommandClient::link_ok, py::arg("now_s") = -1.0,
             "True if telemetry arrived within telemetry_timeout_s.")
        .def("telemetry_age", &CommandClient::telemetry_age, py::arg("now_s") = -1.0)
        .def_property_readonly("stats", &CommandClient::stats)
        .def_property_readonly("last_seq", &CommandClient::last_seq)
        .def_static("now", &CommandClient::now);
}

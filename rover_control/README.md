# rover_control — CU Rover command link

The **commands side** of the rover's radio link: everything that gets commands from the base
station to the rover safely, and status back. A **C++17 core** built with **CMake**, exposed to
**Python** with **pybind11**. (Video is a separate project.)

```
 Base station (Windows/any OS)                          Rover (Jetson Orin, ROS 2)
 ┌──────────────────────────────┐   UDP 5005 over   ┌──────────────────────────────────┐
 │ gamepad / GUI / scripts      │   the radio       │ comms bridge node (TODO)          │
 │   └ CommandClient (C++)  ────┼──────────────────▶│   └ RoverController (C++)        │
 │     drive() estop() ...      │◀──────────────────┼─    e-stop, link loss, modes      │
 └──────────────────────────────┘   telemetry 5006  │   └▶ /drive/cmd_vel → drive team  │
                                                     └──────────────────────────────────┘
 For laptop testing, `rover-node` (the virtual rover) stands in for the right-hand side.
```

**Driving the real rover:** see [ros/README.md](ros/README.md) for the temporary ROS 2 bridge.

**→ See [COMMANDS.md](COMMANDS.md) for every command, button, mode and rover state.**

## Why C++ + pybind11?

| Layer | Language | Why |
|---|---|---|
| Command functions (`CommandClient`), packet protocol, CRC, e-stop, watchdog, modes | C++ (`src/`, `include/`) | Deterministic, fast, unit-tested, reusable in the rover's ROS 2 node |
| Gamepad, sockets, serial, CLI tools | Python (`python/rover/`) | Fast to write and change at competition |
| Glue | pybind11 (`python/bindings.cpp`) | Python calls the C++ classes as if they were native Python |

CMake builds the C++ library, the C++ tests, and the Python module from one `CMakeLists.txt`.

## Quick start

**New Windows PC?** Double-click `tools\setup_windows.bat`. It installs Python and the C++
Build Tools if missing (via winget), sets up `.venv`, builds everything and runs the tests.
For the two-PC cable test, also double-click `tools\network_setup.bat` on each PC
(sets the static IP, Private network profile and firewall rules; `network_undo.bat` reverts).

Manual setup:

Requirements: Python 3.9+, a C++17 compiler (gcc/clang/MSVC), CMake 3.18+.

```bash
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install ".[test]"                  # builds C++ + bindings via scikit-build-core
pytest tests/                          # Python tests (exercise the bindings)

# Try the full pipeline on one machine (two terminals):
rover-node                             # terminal 1: virtual rover
rover-base --rover-ip 127.0.0.1 --fake # terminal 2: base station with scripted input
```

Use the command functions from your own Python code:

```python
from rover import connect, Mode
rover = connect("127.0.0.1")
rover.set_mode(Mode.Teleop)
rover.drive(0.5, 0.0)      # linear, angular (+ = turn left, ROS convention)
rover.estop()
```

With a real gamepad: `pip install ".[gamepad]"`, then `rover-base` (defaults to the rover PC at 10.10.10.5).
Run `rover-base --print-axes` first — axis/button numbers differ between controllers and OSes.

### In VS Code

Open the `rover_control` folder, install the recommended extensions when prompted, select the
`.venv` interpreter, then run the **Install** task once (`Ctrl+Shift+P` → *Tasks: Run Task*).
`Ctrl+Shift+P` → *Tasks: Run Test Task* runs C++ and Python tests; the Run and Debug panel has
**Rover + Base station (fake input)** to launch both sides with breakpoints.
Python edits apply immediately (editable install); after C++ edits, re-run the Install task.

### C++-only build (for C++ development / debugging)

```bash
cmake -S . -B build -DROVER_BUILD_PYTHON=OFF
cmake --build build
ctest --test-dir build --output-on-failure
```

## CU Rover network setup

Defaults match the Comms Team's *Groove 52 ac* setup doc (MikroTik radios bridging Ethernet on
2.4 GHz, channel 6, 20 MHz):

| Device | Radio IP | PC static IP |
|---|---|---|
| Ground station | 10.10.10.1 (AP bridge) | 10.10.10.4 |
| Rover | 10.10.10.2 (station bridge) | **10.10.10.5** ← Jetson Orin (confirm) |
| Drone | 10.10.10.3 (station bridge) | 10.10.10.6 |

```bash
# on the ground-station PC (10.10.10.4)
rover-base
```

Run `ping 10.10.10.5` from the GS PC first; if that fails it's a radio/IP problem, not this code.

**Two transports, same protocol:**

- **UDP over the Groove link** (default). Control packets are marked DSCP EF so a RouterOS
  queue policy can prioritise them over camera video sharing the 2.4 GHz link (bitrate
  sheet estimates ~42 Mbps for 720p30 — drive commands are only ~3 kbps).
- **Serial radio** such as the **RFD900 (900 MHz)** listed in the comms plan:
  `rover-node --radio-serial /dev/ttyUSB0` and `rover-base --serial COM5`. The C++
  `StreamDecoder` re-syncs on corrupted/split bytes. A 900 MHz control link with video on
  2.4 GHz is a good split: losing video then doesn't lose the e-stop.

**Part 97 (amateur radio) reminders** — the setup doc runs the radios under ham rules:
no encryption and no codes meant to obscure meaning. This protocol is unencrypted and fully
documented below; keep it that way and keep this README with the code. Station ID by callsign
(SSID/radio name) is handled on the radio side — confirm with your licensed control operator.

## Gamepad mapping (default)

| Input | Action |
|---|---|
| Left stick Y / Right stick X | Forward-back (linear) / turn (angular) |
| **B** | **E-STOP** (latches) |
| START | Release e-stop |
| A / X / BACK | Teleop / Autonomous / Idle mode |

Ctrl+C on the base station sends an e-stop before exiting.

## Safety behaviour (read this before driving real hardware)

- **E-stop latches.** Output stays at zero until an explicit release, and the command is reset on release.
  An e-stop is honoured even if its packet arrives out of order.
- **Link-loss watchdog.** No valid packet for 0.5 s (`--timeout`) → output zero. Drive commands are
  sent at 20 Hz, so ~10 lost packets in a row trips it. The drive team's `drive_controller` has its
  own command timeout as a second layer.
- **Stale/corrupt packets dropped** via sequence numbers (wrap-safe) and CRC-16.
- **Autonomous mode ignores base-station drive commands** — URC prohibits teleop during autonomy.
  The nav stack drives through `set_autonomy_command()` and must keep publishing (0.3 s timeout).
- **Bad values can't cause motion:** every drive value is clamped to ±1 and NaN becomes 0, on both ends.
- **Speed caps and acceleration limits** belong to the drive team's `drive_controller` (YAML config).

Software stops are *not* a substitute for the physical e-stop/kill switch the URC rules require —
that must cut motor power in hardware.

## URC notes to verify against the current rules

The [2027 rules PDF](https://urc.marssociety.org/home/requirements-guidelines) is authoritative; check it and the
Q&A page for anything below:

- **Autonomy indicator LED:** historically red = autonomous, blue = teleoperation, flashing green =
  arrived. Mapped in `led_for_mode()` (`src/controller.cpp`).
- **Radio frequencies/power:** rules restrict which bands (e.g. 900 MHz, 2.4 GHz, 5 GHz) and
  channels you may use. This code is radio-agnostic: it only needs an IP link (e.g. Ubiquiti
  bridges) — swap `UdpLink` if your radio is serial-only.
- **E-stop and autonomy rules** as described above.

## Project layout

```
CMakeLists.txt            build for C++ lib, tests, and Python module
pyproject.toml            lets `pip install .` drive CMake (scikit-build-core)
include/rover/            public C++ headers
  protocol.hpp            binary packet format + CRC
  command_client.hpp      CommandClient: the command functions (base station)
  controller.hpp          RoverController: rover-side safety gate
  shaping.hpp             joystick deadband / expo
  safety.hpp              watchdog
src/                      C++ implementations
python/bindings.cpp       pybind11 bindings -> rover._core
python/rover/             Python package
  client.py               connect(): CommandClient on a UDP or serial link
  base_station.py         rover-base (gamepad -> CommandClient)
  rover_node.py           rover-node (virtual rover for testing)
  link.py                 UDP and serial transports
  bridge_core.py          rover-side bridge logic (ROS-free, tested)
ros/comms_bridge_node.py  temporary ROS 2 node: UDP -> /drive/cmd_vel
tests/                    C++ (ctest) and Python (pytest) tests
archive/                  old motor/mixing code, now owned by the drive team (not built)
```

## Wire protocol

`[AB CD][ver][type][seq u16][len][payload][crc16]`, little-endian, CRC-16/CCITT-FALSE over
ver..payload. Protocol version 2. Message types: Heartbeat(1), Drive(2: linear f32, angular f32),
EStop(3: u8), SetMode(4: u8), Telemetry(5: linear f32, angular f32, mode u8, estop u8, battery f32).
Drive values follow ROS conventions (+angular = turn left). Bump `kProtocolVersion` if you change it.

## Next steps / TODO

- [ ] Step 3: move IPs, ports, timeouts and stick settings into `config/radios/link_config.yaml`.
- [ ] Step 4: `comms` ROS 2 package with a bridge node: UDP → RoverController → `/drive/cmd_vel`,
      `/comms/estop`, `/comms/mode`, `/comms/led`; `/autonomy/cmd_vel` in Autonomous mode.
- [ ] Agree gamepad layout with the drive team (their xbox node turns with the left stick).
- [ ] Agree port plan and link priority with the video teammate; camera-control commands
      (switch camera, quality) go over this link.
- [ ] More command functions as teams need them: arm, science, camera.
- [ ] Measure real latency/loss over the Groove link and tune timeouts.
- [ ] Decide control link: Groove 2.4 GHz vs RFD900 900 MHz (or both, with failover).

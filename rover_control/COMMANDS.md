# Rover Commands & States

Quick reference for the command link: the command functions, the programs you can run,
every button and message, and every state the rover can be in.

---

## 0. Before anything: open a terminal and activate the environment

Every new terminal in VS Code (**View → Terminal**, or **+** for another one):

```powershell
.venv\Scripts\activate          # Windows  (prompt should start with (.venv))
source .venv/bin/activate       # Mac / Linux
```

If PowerShell says scripts are disabled: `Set-ExecutionPolicy -Scope Process RemoteSigned`, then try again.

The virtual rover needs **two terminals**: one runs the rover, the other runs the base station.

---

## 1. Command functions (`CommandClient`)

The core of the commands side. Written in C++ (`include/rover/command_client.hpp`), and
available in Python through pybind11. Anything that needs to command the rover (the gamepad
program, a GUI, a test script, an arm tool) uses these functions.

```python
from rover import connect, Mode

rover = connect("127.0.0.1")       # UDP; default address is the rover PC 10.10.10.5
# rover = connect(serial="COM5")   # or an RFD900 serial modem

rover.set_mode(Mode.Teleop)
rover.drive(0.5, 0.0)              # call ~20 times a second while driving
rover.poll()                       # read whatever the rover sent back
print(rover.telemetry(), rover.link_ok())
rover.estop()
rover.close()
```

| Function | What it does | Sent |
|---|---|---|
| `drive(linear, angular)` | Drive command. `linear` -1 (reverse) … +1 (forward). `angular` -1 (turn right) … +1 (**turn left**). Clamped to ±1; NaN becomes 0. | once |
| `stop()` | Same as `drive(0, 0)` | once |
| `estop()` | Engage the e-stop. Latches on the rover. | 3× |
| `release_estop()` | Release a latched e-stop | 3× |
| `set_mode(mode)` | `Mode.Idle`, `Mode.Teleop`, `Mode.Autonomous`, `Mode.Arrived` | 3× |
| `heartbeat()` | Keep the link alive without driving | once |
| `poll()` | Read everything the rover sent back. Returns how many telemetry packets arrived. | n/a |
| `telemetry()` | Latest rover report (`mode`, `estop`, `linear`, `angular`, `battery_v`), or `None` | n/a |
| `link_ok()` | `True` if telemetry arrived in the last 1 s | n/a |
| `telemetry_age()` | Seconds since the last telemetry, or -1 if none yet | n/a |
| `stats` | `packets_sent`, `telemetry_received`, `corrupt` | n/a |
| `close()` | Close the link | n/a |

**Sign convention:** `linear`/`angular` follow ROS (REP-103), so they map directly onto
`geometry_msgs/Twist` on `/drive/cmd_vel`: `linear` → `linear.x`, `angular` → `angular.z`.
**Positive angular means turn left.**

"Sent 3×" means safety-critical commands are repeated in case the radio drops one. Each copy
has its own sequence number, and repeats are harmless.

Without a real link (for tests), pass your own send/receive functions:
`CommandClient(send=my_send, recv=my_recv)`. See `tests/test_bindings.py`.

---

## 2. Programs

| Program | Runs on | What it does |
|---|---|---|
| `rover-base` | Base-station PC (or your laptop) | Reads the gamepad (or fake input) and uses `CommandClient` to command the rover 20 times a second |
| `rover-node` | Your laptop (for testing) | **Virtual rover**: receives commands, runs the safety logic, prints the drive command it *would* send to `/drive/cmd_vel` |

On the real rover, a ROS 2 node in the `comms` package will do `rover-node`'s job and publish
to `/drive/cmd_vel` for the drive team's `drive_controller`.

### `rover-base` options

| Option | Default | What it does |
|---|---|---|
| *(none)* | | Real gamepad, sends to the rover PC at `10.10.10.5` |
| `--rover-ip 127.0.0.1` | `10.10.10.5` | Where the rover is. **Use `127.0.0.1` when both run on your laptop.** |
| `--fake` | off | No gamepad needed: scripted stick movement, starts in Teleop |
| `--mode teleop` | Teleop with `--fake`, else none | Put the rover in a mode on startup: `idle`, `teleop`, `autonomous`, `arrived` |
| `--release` | off | Release a latched e-stop on startup |
| `--print-axes` | off | Print raw gamepad axis/button numbers (to check the mapping) |
| `--serial COM5` | UDP | Send over a serial radio (RFD900) instead of the network |
| `--baud 57600` | `57600` | Serial radio baud rate |
| `--rover-port 5005` | `5005` | Rover's UDP port |
| `--listen-port 5006` | `5006` | Port to receive telemetry on |
| **Ctrl+C** | | Sends an **e-stop** to the rover, then exits |

### `rover-node` options (virtual rover)

| Option | Default | What it does |
|---|---|---|
| *(none)* | | Listen on UDP port 5005 |
| `--timeout 1.0` | `0.5` | Seconds without a packet before the rover stops (`LinkLost`) |
| `--demo-autonomy` | off | Fake autonomy stack: drives forward at 30% while in Autonomous mode |
| `--radio-serial COM4` | UDP | Receive commands from a serial radio (RFD900) instead of the network |
| `--baud 57600` | `57600` | Serial radio baud rate |
| `--port 5005` | `5005` | UDP port to listen on |
| `--base-port 5006` | `5006` | UDP port to send telemetry back to |

Add `--help` to either program to see its options.

### Build & test commands

| Command | When to use it |
|---|---|
| `tools\setup_windows.bat` (double-click) | Set up a new Windows PC from scratch: Python, C++ Build Tools, `.venv`, build, tests |
| `tools\network_setup.bat` (double-click) | Two-PC cable test: static IP (rover 10.10.10.5 / base 10.10.10.4), Private profile, firewall. `network_undo.bat` reverts |
| `python -m pip install -e ".[test]"` | First time, and **after changing any C++ file** (`src\`, `include\`, `python\bindings.cpp`) |
| `python tools\run_cpp_tests.py` | Run the C++ tests |
| `python -m pytest tests` | Run the Python tests |
| `pip install ".[gamepad]"` | Add gamepad support (pygame) |
| `pip install ".[serial]"` | Add serial support (pyserial) for the RFD900 |

Python file changes need no rebuild: just run again.

---

## 3. Gamepad controls

Xbox-style layout. Button numbers vary by controller and OS, so check yours with `rover-base --print-axes`.

| Control | Action | Function called |
|---|---|---|
| Left stick up/down | Forward / reverse | `drive(linear, …)` |
| Right stick left/right | Turn | `drive(…, angular)` |
| **B** | **E-STOP** (latches) | `estop()` |
| START | Release e-stop | `release_estop()` |
| A | Teleop mode | `set_mode(Mode.Teleop)` |
| X | Autonomous mode | `set_mode(Mode.Autonomous)` |
| BACK | Idle mode | `set_mode(Mode.Idle)` |

Stick shaping before sending: 8% deadband, 0.3 expo (finer control near centre), turning scaled to 70%.
There is no button for Arrived; use `--mode arrived`, or let the autonomy stack set it.

The drive team's `xbox_controller_node` turns with the **left** stick instead. Agree on one layout.

---

## 4. Rover modes

The rover is always in exactly one **mode**. The mode decides the LED colour and who is allowed to drive.

| Mode | LED | Who drives | Output | How to enter |
|---|---|---|---|---|
| **Idle** | Off | Nobody | Zero | Rover just started · BACK button · `--mode idle` |
| **Teleop** | Blue | Base-station sticks | Follows the sticks | A button · `--fake` · `--mode teleop` |
| **Autonomous** | Red | Rover's autonomy stack only. **Stick commands are ignored** (URC rule) | Follows autonomy commands | X button · `--mode autonomous` |
| **Arrived** | Flashing green | Nobody | Zero | `--mode arrived` (later: set by autonomy on reaching a target) |

LED colours are the historical URC convention. Check them against the 2027 rules.

Notes:
- **Changing mode** clears old stick/autonomy commands, so the rover never carries motion from one mode into another.
- **The mode survives a link loss.** If the radio drops in Teleop, the rover stops, but it is still in Teleop when the link comes back.

---

## 5. Drive status (what the virtual rover prints)

About 5 times a second the virtual rover prints the command it would publish to `/drive/cmd_vel`:

```
[cmd_vel] linear=+0.40 angular=+0.11 DRIVE                LED=Blue
[cmd_vel] linear=+0.00 angular=+0.00 STOP(EStop)          LED=Blue
```

| Status | Meaning | How to get out of it |
|---|---|---|
| `DRIVE` | Commands are passed through to the drive system | n/a |
| `STOP(EStop)` | E-stop is latched | `release_estop()`: START button, or restart `rover-base` with `--release` |
| `STOP(LinkLost)` | No valid packet for 0.5 s (Teleop, or Autonomous by default) | Get the base station sending again. Recovers automatically |
| `STOP(NoAutonomyCmd)` | In Autonomous, but the autonomy stack hasn't sent a command in 0.3 s | Start the autonomy stack (or `rover-node --demo-autonomy`) |
| `STOP(Idle)` | Mode is Idle or Arrived, so it holds still on purpose | Switch to Teleop or Autonomous |

When stopped, the output is always exactly zero. When the virtual rover exits it prints
`link stats: accepted=… corrupt=… stale=…` (packets used / failed the CRC check / arrived out of
order and were ignored).

### Which stop wins

If several apply at once, the rover reports the first match:

1. **E-stop latched** → `STOP(EStop)` (always wins, in every mode)
2. Mode is **Teleop**: link lost → `STOP(LinkLost)`, otherwise `DRIVE`
3. Mode is **Autonomous**: link lost → `STOP(LinkLost)`; no autonomy command → `STOP(NoAutonomyCmd)`; otherwise `DRIVE`
4. Mode is **Idle** or **Arrived** → `STOP(Idle)`

---

## 6. E-stop rules

- **Latching:** once engaged it stays engaged through reconnects, mode changes and restarts of the base station. Only an explicit release clears it.
- **Never ignored:** an e-stop is obeyed even if its packet arrives late or out of order.
- **Release is safe:** releasing resets the drive command to zero, so the rover doesn't jump to the last stick position. The mode is unchanged.
- **Ctrl+C on the base station** sends an e-stop before exiting.
- The **physical e-stop** (required by URC) must cut motor power in hardware; this software stop is in addition to it, not instead.

---

## 7. State diagram

```
                      SetMode(Teleop)                    SetMode(Autonomous)
          ┌────────┐ ───────────────▶ ┌──────────┐ ◀──────────────────▶ ┌──────────────┐
 start ──▶│  IDLE  │                  │  TELEOP  │                      │  AUTONOMOUS  │
          │ LED off│ ◀─────────────── │ LED blue │                      │   LED red    │
          └────────┘   SetMode(Idle)  └──────────┘                      └──────┬───────┘
               ▲                         (any mode can switch to any other)      │ SetMode(Arrived)
               │                                                                 ▼
               │                     SetMode(Idle)                       ┌──────────────┐
               └──────────────────────────────────────────────────────── │   ARRIVED    │
                                                                         │ LED flashing │
                                                                         │    green     │
                                                                         └──────────────┘

 Layered on top of ANY mode:
   E-stop engaged ──▶ STOP(EStop) until EStop(release)
   No packets 0.5 s ──▶ STOP(LinkLost) until packets resume (Teleop / Autonomous)
```

---

## 8. Try every state on your laptop

Terminal 1 runs the virtual rover, terminal 2 runs the base station. Stop each with Ctrl+C before the next row.

| To see… | Terminal 1 | Terminal 2 | Expected rover output |
|---|---|---|---|
| Idle | `rover-node` | *(nothing yet)* | `STOP(Idle) LED=Off` |
| Teleop driving | `rover-node` | `rover-base --rover-ip 127.0.0.1 --fake` | `DRIVE LED=Blue`, linear/angular changing |
| E-stop | *(keep running)* | press **Ctrl+C** | `STOP(EStop)` |
| E-stop release | *(keep running)* | wait 1 s, then `rover-base --rover-ip 127.0.0.1 --fake --release` | back to `DRIVE` |
| Link lost | `rover-node` | `rover-base --rover-ip 127.0.0.1 --fake`, then close terminal 2 with the 🗑 icon (no Ctrl+C) | `STOP(LinkLost)` |
| Autonomous, no autonomy stack | `rover-node` | `rover-base --rover-ip 127.0.0.1 --fake --mode autonomous` | `STOP(NoAutonomyCmd) LED=Red` (sticks ignored) |
| Autonomous driving | `rover-node --demo-autonomy` | `rover-base --rover-ip 127.0.0.1 --fake --mode autonomous` | `DRIVE LED=Red`, `linear=+0.30` |
| Arrived | `rover-node` | `rover-base --rover-ip 127.0.0.1 --fake --mode arrived` | `STOP(Idle) LED=FlashingGreen` |
| Back to Idle | `rover-node` | `rover-base --rover-ip 127.0.0.1 --fake --mode idle` | `STOP(Idle) LED=Off` |

The base-station terminal prints what it sent (`-> mode Teleop`), when the link comes up or
goes down, and about once a second what the rover reports
(`<- rover mode=Teleop estop=False linear=+0.40 angular=+0.11`).

After an e-stop, wait about 1 second before restarting `rover-base`. The rover only accepts a
"fresh start" from a base station after the old link has timed out.

---

## 9. Messages on the wire

What actually travels over the radio. Each one is a small binary packet (9–23 bytes) with a
sequence number and CRC checksum. Protocol version **2**. Full byte layout is in `README.md` → *Wire protocol*.

| # | Message | Direction | Contents | Effect on the rover |
|---|---|---|---|---|
| 1 | `Heartbeat` | base → rover | nothing | Keeps the link alive (resets the 0.5 s timer) |
| 2 | `Drive` | base → rover | linear, angular (-1…1, + angular = left) | New drive command (used in Teleop only). Also keeps the link alive. Sent 20×/s |
| 3 | `EStop` | base → rover | engage / release | Latch or release the e-stop |
| 4 | `SetMode` | base → rover | Idle / Teleop / Autonomous / Arrived | Change mode (clears old commands) |
| 5 | `Telemetry` | rover → base | linear, angular, mode, e-stop, battery V | Status report, 5×/s |

Adding a new command (arm, science, camera switch…): add a message type to `protocol.hpp`/`.cpp`,
a function to `CommandClient`, its binding in `python/bindings.cpp`, and tests.

---

## 10. Timing & default settings

| Setting | Value | Where to change it |
|---|---|---|
| Base station send rate | 20 Hz | `SEND_HZ` in `base_station.py` |
| Virtual rover loop | 50 Hz | `CONTROL_HZ` in `rover_node.py` |
| Telemetry rate | 5 Hz | `TELEMETRY_HZ` in `rover_node.py` |
| Link-loss timeout (rover) | 0.5 s | `rover-node --timeout` / `ControllerConfig.watchdog_timeout_s` |
| Autonomy command timeout | 0.3 s | `ControllerConfig.autonomy_timeout_s` |
| Telemetry timeout (`link_ok`) | 1.0 s | `ClientConfig.telemetry_timeout_s` |
| Critical-command repeats | 3 | `ClientConfig.critical_repeats` |
| Stick deadband / expo / turn scale | 8% / 0.3 / 70% | `DEADBAND`, `EXPO`, `TURN_SCALE` in `base_station.py` |
| Network | Rover PC `10.10.10.5`, GS PC `10.10.10.4`, UDP 5005/5006 | `python/rover/__init__.py` |

Speed caps and acceleration limits are handled by the drive team's `drive_controller`
(`config/drive/drive_config.yaml` in 2026-Rover-Code).

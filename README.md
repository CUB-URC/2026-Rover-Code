# 2026 CUB-URC Rover Code

Complete software stack for the CUB-URC rover platform, including drive control, autonomous navigation, science payload management, and drone integration.

## Quick Start

### Prerequisites

- Docker and Docker Compose installed
- Access to the rover's networks (when deploying on hardware)

### Running in Development

Navigate to the `./docker` folder and launch a development container:

```bash
cd docker
docker compose run dev
```

Inside the container:
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Deploying to Hardware

**For Jetson Orin Nano (main rover):**
```bash
docker compose run orin
```

**For Jetson Nano (secondary units):**
```bash
docker compose run nano
```

> **Note:** Host OS doesn't matter—all development and deployment happens inside Docker containers.

## System Architecture

### Computing Units

| Device | Role | Specs |
|--------|------|-------|
| Jetson Orin Nano Super Dev Kit | Main rover controller | 8-core CPU, 8GB RAM |
| Jetson Nano Dev Kit (×2) | Coprocessors | 4-core CPU, 4GB RAM each |
| Jetson Nano Dev Kit | Drone processing | 4-core CPU, 4GB RAM |

### Sensors & Actuators

| Component | Quantity | Specs |
|-----------|----------|-------|
| Drive Motors | 6× | Saturn 5304 Series Planetary Gear Motors |
| Motor Controllers | 2× | BILDA 2x40A Motor Controller (H-Bridge) |
| Main Camera | 1× | ZED 2 Stereoscopic Camera |
| Camera SDK | — | ZED SDK 5.1 |

## Building and Testing

### Build the ROS2 Workspace

```bash
cd ros2_ws
colcon build --symlink-install
```

### Source the Setup

```bash
source install/setup.bash
```

### Run Tests

```bash
ros2 run drive drive_controller_sim  # Test drive in simulation mode
```

## Package Overview

- **drive** - Motor control and PWM management
- **autonomy** - Path planning and navigation
- **science** - Science payload control
- **comms** - Communication and radio management
- **drone** - Drone integration and control
- **simulation** - Physics simulation environment

## Documentation

See the `docs/` folder for detailed guides:
- [Coding Standards](docs/coding_standards.md) - Code style and conventions
- [Drive Hardware Setup](docs/drive_hardware_setup.md) - Motor wiring and PWM configuration
- [Network Configuration](docs/network.md) - Network setup and connectivity
- [Hardware Schematics](hardware/schematics/) - Circuit diagrams and pinouts
- [Onboarding Guide](docs/onboarding.md) - Getting started with development

## Development Workflow

1. **Create a feature branch** from `main`
2. **Write tests** for new functionality
3. **Build and test** locally in Docker
4. **Submit a pull request** with detailed description
5. **Code review** before merging

## Troubleshooting

**Build fails with missing dependencies:**
```bash
rm -rf build install log
colcon build --symlink-install
```

**ROS2 tools not found:**
```bash
source /opt/ros/humble/setup.bash
```

**Hardware not responding:**
See [Drive Hardware Setup](docs/drive_hardware_setup.md#troubleshooting) for diagnostics.

## License

See [LICENSE](LICENSE) file for details.


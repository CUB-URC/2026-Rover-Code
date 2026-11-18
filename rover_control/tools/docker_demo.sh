#!/usr/bin/env bash
# run the comms bridge + a fake base station inside the ros humble docker image and
# print what lands on /drive/cmd_vel. works on mac (no gamepad, uses --fake).
#
#   ./rover_control/tools/docker_demo.sh            (from the repo root, docker running)
#
# uses the plain ros:humble-ros-base image (override with IMAGE=docker-dev etc). rover_control is mounted
# read-only and copied inside the container, so nothing gets built into the repo.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE="${IMAGE:-ros:humble-ros-base}"

docker run --rm -v "$ROOT/rover_control:/src:ro" "$IMAGE" bash -c '
set -e
source /opt/ros/humble/setup.bash
# older images have no pip, and ubuntu 22.04s stock "packaging" breaks the scikit-build build
command -v pip3 >/dev/null || { apt-get update -qq && apt-get install -y -qq python3-pip >/dev/null; }
apt-get install -y -qq python3-dev cmake >/dev/null 2>&1 || true
cp -r /src /tmp/rc && cd /tmp/rc
pip3 install -q -U pip packaging 2>&1 | tail -1
pip3 install -q . 2>&1 | grep -v -i warning | tail -5
export PYTHONUNBUFFERED=1
python3 ros/comms_bridge_node.py > /tmp/bridge.log 2>&1 &
sleep 4
rover-base --rover-ip 127.0.0.1 --fake > /tmp/base.log 2>&1 &
sleep 3
echo "=== /drive/cmd_vel linear.x / angular.z"
timeout 4 ros2 topic echo --no-daemon /drive/cmd_vel --field linear.x | grep -v -- --- | head -8 | tr "\n" " "; echo
timeout 3 ros2 topic echo --no-daemon /drive/cmd_vel --field angular.z | grep -v -- --- | head -5 | tr "\n" " "; echo
echo "=== status"
for t in mode led link_ok; do echo -n "$t: "; timeout 3 ros2 topic echo --no-daemon /comms/$t --once | head -1; done
echo "=== killing the base station (link loss)"
pkill -9 -f rover-base; sleep 2
for t in stop_reason link_ok; do echo -n "$t: "; timeout 3 ros2 topic echo --no-daemon /comms/$t --once | head -1; done
echo -n "cmd_vel: "; timeout 3 ros2 topic echo --no-daemon /drive/cmd_vel --once --field linear.x | head -1
'

#!/usr/bin/env python3
"""comms_bridge: ROS 2 node that lets rover-base drive the real rover.

    base station (rover-base, gamepad)  --UDP 5005-->  this node  --> /drive/cmd_vel
                                        <--UDP 5006--  (telemetry)     (drive_controller)

Publishes
    /drive/cmd_vel      geometry_msgs/Twist  50 Hz. linear.x, angular.z (+ = turn left).
                                             Exactly zero whenever stopped (e-stop, link lost, Idle...).
    /comms/estop        std_msgs/Bool        5 Hz
    /comms/link_ok      std_msgs/Bool        5 Hz  (base station heard within the timeout)
    /comms/mode         std_msgs/String      5 Hz  Idle / Teleop / Autonomous / Arrived
    /comms/stop_reason  std_msgs/String      5 Hz  None / EStop / LinkLost / Idle / NoAutonomyCmd
    /comms/led          std_msgs/String      5 Hz  Off / Red / Blue / FlashingGreen
Subscribes
    /autonomy/cmd_vel   geometry_msgs/Twist  used only in Autonomous mode
Services
    /emergency_stop     std_srvs/Trigger     engage the e-stop from the rover side
                                             (release only from the base station)
Parameters (defaults)
    port 5005, base_port 5006, rate_hz 50.0, link_timeout_s 0.5, autonomy_timeout_s 0.3,
    max_linear 0.3, max_angular 0.3   <- start slow; raise once the rover behaves

Run on the Jetson (ROS 2 Humble sourced, rover_control pip-installed):
    source /opt/ros/humble/setup.bash
    python3 ros/comms_bridge_node.py
    python3 ros/comms_bridge_node.py --ros-args -p max_linear:=0.5 -p max_angular:=0.5

IMPORTANT: only one node may publish /drive/cmd_vel. Do not run xbox_controller_node or
`ros2 topic pub /drive/cmd_vel` at the same time.

Temporary script for testing. The long-term plan is a C++ node in the `comms` package.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from rover import ControllerConfig, StopReason
from rover.bridge_core import BridgeCore


class CommsBridge(Node):
    def __init__(self):
        super().__init__("comms_bridge")
        p = self.declare_parameter
        port = p("port", 5005).value
        base_port = p("base_port", 5006).value
        rate_hz = float(p("rate_hz", 50.0).value)
        cfg = ControllerConfig()
        cfg.watchdog_timeout_s = float(p("link_timeout_s", 0.5).value)
        cfg.autonomy_timeout_s = float(p("autonomy_timeout_s", 0.3).value)
        max_linear = float(p("max_linear", 0.3).value)
        max_angular = float(p("max_angular", 0.3).value)

        self.core = BridgeCore(port, base_port, cfg, max_linear, max_angular)

        self.cmd_pub = self.create_publisher(Twist, "/drive/cmd_vel", 10)
        self.estop_pub = self.create_publisher(Bool, "/comms/estop", 10)
        self.link_pub = self.create_publisher(Bool, "/comms/link_ok", 10)
        self.mode_pub = self.create_publisher(String, "/comms/mode", 10)
        self.reason_pub = self.create_publisher(String, "/comms/stop_reason", 10)
        self.led_pub = self.create_publisher(String, "/comms/led", 10)
        self.create_subscription(Twist, "/autonomy/cmd_vel", self.on_autonomy, 10)
        self.create_service(Trigger, "/emergency_stop", self.on_estop_service)

        self.create_timer(1.0 / rate_hz, self.tick)
        self.create_timer(0.2, self.publish_status)
        self.last_state = None
        self.last_out = None

        self.get_logger().info(
            f"comms_bridge listening on UDP {port}, publishing /drive/cmd_vel at {rate_hz:.0f} Hz, "
            f"max_linear={self.core.max_linear} max_angular={self.core.max_angular}")

    def on_autonomy(self, msg: Twist):
        self.core.autonomy_command(msg.linear.x, msg.angular.z, time.monotonic())

    def on_estop_service(self, request, response):
        self.core.local_estop(time.monotonic())
        response.success = True
        response.message = "E-stop engaged. Release it from the base station (START)."
        self.get_logger().warn("E-stop engaged via /emergency_stop service")
        return response

    def tick(self):
        now = time.monotonic()
        res = self.core.step(now)
        self.last_out = res.drive

        msg = Twist()
        msg.linear.x = float(res.linear)
        msg.angular.z = float(res.angular)
        self.cmd_pub.publish(msg)          # always publish, zeros included

        state = (self.core.ctrl.mode.name, res.drive.reason.name, self.core.ctrl.estopped)
        if state != self.last_state:       # log changes only
            self.get_logger().info(
                f"mode={state[0]} status={'DRIVE' if res.drive.enabled else 'STOP(' + state[1] + ')'}"
                f" estop={state[2]}")
            self.last_state = state

    def publish_status(self):
        now = time.monotonic()
        ctrl = self.core.ctrl
        self.estop_pub.publish(Bool(data=bool(ctrl.estopped)))
        self.link_pub.publish(Bool(data=bool(ctrl.link_ok(now))))
        self.mode_pub.publish(String(data=ctrl.mode.name))
        reason = self.last_out.reason if self.last_out else StopReason.Idle
        led = self.last_out.led.name if self.last_out else "Off"
        self.reason_pub.publish(String(data="None" if reason == StopReason.None_ else reason.name))
        self.led_pub.publish(String(data=led))

    def stop_motors(self):
        try:
            for _ in range(3):
                self.cmd_pub.publish(Twist())
        except Exception:
            pass  # context may already be shut down; drive_controller's timeout covers it


def main():
    rclpy.init()
    node = CommsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.core.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""autonomous_nav_controller.py

High level autonomous nav controller for CUB-URC rover.

-- MODES --

1. waypoint => nav to single GPS or local coord
2. search   => lawnmower ssearch pattern around GPS coord
3. demo     => nav to hardcoded local (non-gps) goal

run: ```ros2 run autonomy autonomous_nav_controller --ros-args -p mode:=<mode>```

* publishes: /mission.nav_status    (std_msgs/String)   curr mission state

-- PARAMS --

* mode          (str)   waypoint | search | demo
* goal_lat      (float) target latitude
* goal_lon      (float) target longitude
* search_rad    (float) half-side of search square in meters (def: 10.0)
* search_step   (float) lawnmower row spacing in meters (def: 2.0)
* use_gps       (bool) whether to use GPS for localization (def: false)

"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class AutonomousNavController(Node):
    def __init__(self):
        super().__init__("autonomous_nav_controller")

        # --- PARAMETERS ---
        self.mode = (
            self.declare_parameter("mode", "demo").get_parameter_value().string_value
        )
        self.goal_lat = (
            self.declare_parameter("goal_lat", 0.0).get_parameter_value().double_value
        )
        self.goal_lon = (
            self.declare_parameter("goal_lon", 0.0).get_parameter_value().double_value
        )
        self.search_rad = (
            self.declare_parameter("search_rad", 10.0)
            .get_parameter_value()
            .double_value
        )
        self.search_step = (
            self.declare_parameter("search_step", 2.0)
            .get_parameter_value()
            .double_value
        )
        self.use_gps = (
            self.declare_parameter("use_gps", False).get_parameter_value().bool_value
        )

        # --- PUBLISHERS ---
        self.status_pub = self.create_publisher(String, "/mission/nav_status", 10)

        # --- NAV2 ---
        self.navigator = BasicNavigator()

        self.get_logger().info(
            f"AutonomousNavController started | mode={self.mode} | use_gps={self.use_gps}"
        )

        def _publish_status(self, msg: str):
            self.get_logger().info(msg)
            self.status_pub.publish(String(data=msg))

        def _build_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
            pose = PoseStamped()
            pose.header.frame_id = "map" if self.use_gps else "odom"  # ?
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # get quat from yaw (rotate around z only)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            return pose

        def _lawnmower_wps(
            self, cx: float, cy: float, r: float, step: float
        ) -> list[PoseStamped]:
            """
            gen lawnmower (boustrophedon) search pattern centered at (cx, cy) with radius r and row spacing step.

            * rows run east to west
            * rover drives north at end of row

            """

            wps = []
            y = cy - r
            row = 0
            eps = 1e-6

            while y <= cy + r + eps:
                if row % 2 == 0:  # even rows => W -> E, yaw=0
                    x_0, x_n, yaw = cx - r, cx + r, 0.0
                else:  # odd rows => E -> W, yaw=pi
                    x_0, x_n, yaw = cx + r, cx - r, math.pi
                wps.append(self._build_pose(x_0, y, yaw))
                wps.append(self._build_pose(x_n, y, yaw))
                y += step
                row += 1
            return wps

        # -- MODES --

        def run_waypoint(self):
            """nav to single GPS or local coord"""
            self._publish_status(
                f"[ WAYPOINT ]: target = ({self.goal_lat:.6f}, {self.goal_lon:.6f}) "
                f"[use_gps={self.use_gps}]"
            )

            if self.use_gps:
                # nav2 GPS waypoint follower converts lat/lon → cartesian internally
                from nav2_msgs.action import FollowGPSWaypoints
                from geographic_msgs.msg import GeoPose
                from builtin_interfaces.msg import Duration as BuiltinDuration

                # use nav2's built-in GPS waypoint action - BasicNavigator exposes followGpsWaypoints() in Iron+
                from geographic_msgs.msg import GeoPoint
                import yaml

                goal = GeoPoint()
                goal.latitude = self.goal_lat
                goal.longitude = self.goal_lon
                goal.altitude = 0.0

                self.navigator.followGpsWaypoints([goal])
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback:
                        self._publish_status(
                            f"[ WAYPOINT ]: navigating... wp {feedback.current_waypoint + 1}/1"
                        )
                    rclpy.spin_once(self, timeout_sec=0.1)
            else:
                # Treat goal_lat/lon as local x/y in the map frame (bench testing)
                goal_pose = self._make_pose(self.goal_lat, self.goal_lon)
                self.navigator.goToPose(goal_pose)
                while not self.navigator.isTaskComplete():
                    rclpy.spin_once(self, timeout_sec=0.1)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self._publish_status("[ WAYPOINT ]: SUCCEEDED :)")
            elif result == TaskResult.CANCELED:
                self._publish_status("[ WAYPOINT ]: CANCELED")
            else:
                self._publish_status(f"[ WAYPOINT ]: FAILED :( (result={result})")

        def run_search(self):
            """run lawnmower search pattern"""
            self._publish_status(
                f"[ SEARCH ]: starting pattern, center=({self.goal_lat:.4f},{self.goal_lon:.4f}) "
                f"radius={self.search_rad}m step={self.search_step}m"
            )

            if self.use_gps:
                # gps:=true => goal_lat/lon is the center of the search area.
                # navsat_transform's datum converts it to (cx, cy) in map frame.
                # use approximation: (0, 0) as datum origin. compute offset via haversine.
                self.get_logger().warn(
                    "GPS search mode: using (0,0) as datum center. "
                    "For accurate GPS search, call /fromLL service to convert lat/lon → map coords."
                )
                cx, cy = (
                    0.0,
                    0.0,
                )  # TODO: replace with robot_localization /fromLL service call
            else:
                cx, cy = self.goal_lat, self.goal_lon  # treat as local coords

            wps = self._lawnmower_wps(cx, cy, self.search_rad, self.search_step)
            self._publish_status(f"[ SEARCH ]: generated {len(wps)} waypoints")

            self.navigator.followWaypoints(wps)
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    n = feedback.current_waypoint + 1
                    self._publish_status(f"[ SEARCH ]: waypoint {n}/{len(wps)}")
                rclpy.spin_once(self, timeout_sec=0.1)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self._publish_status("[ SEARCH ]: SUCCESS :) - pattern complete")
            else:
                self._publish_status(
                    f"[ SEARCH ]: FAILED :( - ended with result={result}"
                )

        def run_demo_mode(self):
            """
            nav to hardcoded local goal for testing purposes.
            doesn't req gps => works with static map→odom TF.
            * drive 2m forward from origin
            """
            self._publish_status(
                "[ DEMO ]: navigating to (2.0, 0.0) in local map frame"
            )
            goal_pose = self._make_pose(2.0, 0.0, yaw=0.0)
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    dist = feedback.distance_remaining
                    self._publish_status(f"[ DEMO ]: {dist:.2f}m remaining")
                rclpy.spin_once(self, timeout_sec=0.1)

            result = self.navigator.getResult()
            self._publish_status(f"[ DEMO ]: result={result}")

        def run(self):
            self._publish_status(f"Waiting on nav2...")
            self.navigator.waitUntilNav2Active()
            self._publish_status("Nav2 is active, begin mission")

            if self.mode == "waypoint":
                self.run_waypoint_mode()
            elif self.mode == "search":
                self.run_search_mode()
            elif self.mode == "demo":
                self.run_demo_mode()
            else:
                self.get_logger().error(
                    f"Unknown mode '{self.mode}'. Please choose: waypoint | search | demo"
                )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.lifecycleShutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

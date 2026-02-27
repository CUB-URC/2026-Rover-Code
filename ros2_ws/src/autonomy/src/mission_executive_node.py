import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from enum import Enum

# Define the states of the mission
class MissionState(Enum):
    IDLE = 0
    NAV_TO_WAYPOINT = 1
    SEARCH_PATTERN = 2
    APPROACH_OBJECT = 3
    DONE = 4

class MissionExecutiveNode(Node):
    def __init__(self):
        super().__init__("mission_executive_node")

        # --- PARAMETERS ---
        self.declare_parameter("use_gps", False)
        self.use_gps = self.get_parameter("use_gps").value
        
        # --- STATE MANAGEMENT ---
        self.state = MissionState.IDLE
        self.target_pose = None  # Pose of the detected object (ArUco/YOLO)
        self.object_detected = False
        
        # --- NAV2 ---
        self.navigator = BasicNavigator()

        # --- SUBSCRIBERS ---
        # Listen for the specific ArUco post or YOLO object you are hunting
        self.create_subscription(
            PoseStamped, 
            "/mission/nav_post_1_pose",  # Change to /mission/object_detection for YOLO
            self.perception_callback, 
            10
        )

        # --- PUBLISHERS ---
        self.status_pub = self.create_publisher(String, "/mission/status", 10)

        # --- TIMER (Control Loop) ---
        # Run the state machine logic at 5Hz
        self.timer = self.create_timer(0.2, self.control_loop)
        
        self.get_logger().info("Mission Executive Node Started")

    def perception_callback(self, msg: PoseStamped):
        """Updates internal state when the target object is seen"""
        self.target_pose = msg
        self.object_detected = True
        # TODO: Add logic to filter noise (e.g., check distance or confidence)

    def control_loop(self):
        """Main State Machine Loop"""
        self._publish_status()

        # --- STATE: IDLE ---
        if self.state == MissionState.IDLE:
            # Wait for Nav2 to be fully active
            # Note: lifecycleStartup() is non-blocking if already active
            self.navigator.lifecycleStartup() 
            
            self.get_logger().info("Nav2 Ready. Starting Mission -> NAV_TO_WAYPOINT")
            self.start_waypoint_nav()
            self.state = MissionState.NAV_TO_WAYPOINT

        # --- STATE: NAV_TO_WAYPOINT ---
        elif self.state == MissionState.NAV_TO_WAYPOINT:
            # PRIORITY 1: If object seen, cancel navigation and approach
            if self.object_detected:
                self.get_logger().info("Object Detected! Cancelling Waypoint -> APPROACH")
                self.navigator.cancelTask()
                self.state = MissionState.APPROACH_OBJECT
                return

            # PRIORITY 2: Check if waypoint reached
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Waypoint Reached. No object yet -> SEARCH_PATTERN")
                    self.start_search_pattern()
                    self.state = MissionState.SEARCH_PATTERN
                else:
                    self.get_logger().warn(f"Waypoint Nav Failed: {result}")
                    self.state = MissionState.DONE

        # --- STATE: SEARCH_PATTERN ---
        elif self.state == MissionState.SEARCH_PATTERN:
            # PRIORITY 1: If object seen, cancel search and approach
            if self.object_detected:
                self.get_logger().info("Object Detected! Cancelling Search -> APPROACH")
                self.navigator.cancelTask()
                self.state = MissionState.APPROACH_OBJECT
                return

            # PRIORITY 2: Check if search pattern finished
            if self.navigator.isTaskComplete():
                self.get_logger().info("Search Complete. Object not found.")
                self.state = MissionState.DONE

        # --- STATE: APPROACH_OBJECT ---
        elif self.state == MissionState.APPROACH_OBJECT:
            # Simple approach: Drive to the last known pose of the tag using Nav2
            # Ideally, you would switch to a 'visual_servoing_node' here for precision
            
            if self.target_pose:
                self.get_logger().info("Sending Nav2 goal to detected object...")
                self.navigator.goToPose(self.target_pose)
                self.target_pose = None # Clear so we don't re-send immediately
            
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Arrived at Object.")
                    self.state = MissionState.DONE
                else:
                    self.get_logger().warn("Approach failed.")
                    self.state = MissionState.DONE

        # --- STATE: DONE ---
        elif self.state == MissionState.DONE:
            pass

    # --- ACTIONS ---

    def start_waypoint_nav(self):
        """Send the robot to the approximate GPS/Map location"""
        # Example: 5 meters forward. In reality, use self.navigator.followGpsWaypoints()
        goal_pose = self._build_pose(5.0, 0.0) 
        self.navigator.goToPose(goal_pose)

    def start_search_pattern(self):
        """Generate and follow a lawnmower pattern"""
        # Center the search on the current location
        # (Simplified: hardcoded 2m square for demo)
        wps = []
        wps.append(self._build_pose(1.0, 1.0))
        wps.append(self._build_pose(1.0, -1.0))
        wps.append(self._build_pose(-1.0, -1.0))
        wps.append(self._build_pose(-1.0, 1.0))
        
        self.navigator.followWaypoints(wps)

    # --- UTILS ---

    def _build_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "map" if self.use_gps else "odom"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_status(self):
        msg = String()
        msg.data = f"State: {self.state.name} | Detected: {self.object_detected}"
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import os


class WaypointLogger(Node):

    def __init__(self):
        super().__init__('waypoint_logger_node')

        # parameters
        self.declare_parameter('logging_file_path', '~/waypoints.yaml')
        self.logging_file_path = os.path.expanduser(
            self.get_parameter('logging_file_path').value)

        # subscriptions
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 1)
        self.current_pose = None
        self.waypoints = []

        self.get_logger().info(f"Waypoint Logger started. Saving to: {self.logging_file_path}")

    def pose_callback(self, msg):
        self.current_pose = msg

    def log_waypoint(self):

        if self.current_pose is None:
            self.get_logger().info("First waypoint logged")
            return

        waypoint = {
            "x": self.current_pose.pose.pose.position.x,
            "y": self.current_pose.pose.pose.position.y,
            "z": self.current_pose.pose.pose.position.z,
            "qx": self.current_pose.pose.pose.orientation.x,
            "qy": self.current_pose.pose.pose.orientation.y,
            "qz": self.current_pose.pose.pose.orientation.z,
            "qw": self.current_pose.pose.pose.orientation.w,
        }

        if waypoint["x"] == 0.0 and waypoint["y"] == 0.0 and waypoint["z"] == 0.0:
            self.get_logger().warning("Default pose (0, 0, 0) detected. Skipping logging.")
            return

        self.waypoints.append(waypoint)  

    
        try:
            with open(self.logging_file_path, 'w') as yaml_file:
                yaml.dump({"waypoints": self.waypoints}, yaml_file, default_flow_style=False)
            self.get_logger().info("Waypoint logged successfully!")
        except Exception as ex:
            self.get_logger().error(f"Error saving waypoint: {str(ex)}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            user_input = input("Press '1' to log waypoint, 'q' to quit: ").strip().lower()
            if user_input == '1':
                node.log_waypoint()
            elif user_input == 'q':
                node.log_waypoint()
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

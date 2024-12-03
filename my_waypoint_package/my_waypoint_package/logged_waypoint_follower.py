import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import yaml
import os


class SequentialWaypointNavigator(Node):

    def __init__(self):
        super().__init__('sequential_waypoint_navigator_node')

        #parameters
        self.declare_parameter('waypoints_file_path', '~/waypoints.yaml')
        self.waypoints_file_path = os.path.expanduser(
            self.get_parameter('waypoints_file_path').value)

        # Load waypoints
        self.waypoints = self.load_waypoints()

        if not self.waypoints:
            self.get_logger().error("No waypoints loaded. Exiting.")
            rclpy.shutdown()
            return

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for NavigateToPose action server...")
        self.action_client.wait_for_server()

        self.get_logger().info("Action server available. Navigating to waypoints sequentially...")
        self.current_waypoint_index = 0
        self.navigate_to_next_waypoint()

    def load_waypoints(self):
        try:
            with open(self.waypoints_file_path, 'r') as yaml_file:
                data = yaml.safe_load(yaml_file)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
            return []

    def navigate_to_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached. Navigation complete.")
            rclpy.shutdown()
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = waypoint['x']
        pose_msg.pose.position.y = waypoint['y']
        pose_msg.pose.position.z = waypoint['z']
        pose_msg.pose.orientation.x = waypoint['qx']
        pose_msg.pose.orientation.y = waypoint['qy']
        pose_msg.pose.orientation.z = waypoint['qz']
        pose_msg.pose.orientation.w = waypoint['qw']

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}...")
        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(
            self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info(f"Feedback: Distance remaining: {feedback.distance_remaining:.2f} meters")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the NavigateToPose action server.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted by the NavigateToPose action server.")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().error(f"Failed to reach waypoint {self.current_waypoint_index + 1}.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SequentialWaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

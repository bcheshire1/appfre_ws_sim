import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(3, self.timer_callback)
        self.x_pos_list = [0.0, 1.0, 2.0, 1.0]
        self.y_pos_list = [0.0, 1.0, 0.0, -1.0]
        self.z_pos = 0.0
        self.current_pose_index = 0
        self.is_goal_reached = True

        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray,
            '/follow_path/_action/status',
            self.goal_status_callback,
            10
        )

    def timer_callback(self):
        if not self.is_goal_reached:
            self.republish_previous_pose()
        else:
            self.publish_next_pose()

    def publish_next_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.x_pos_list[self.current_pose_index]
        pose_msg.pose.position.y = self.y_pos_list[self.current_pose_index]
        pose_msg.pose.position.z = self.z_pos

        if self.current_pose_index > 0:
            # Calculate the yaw angle based on the difference between the current and next pose
            delta_x = pose_msg.pose.position.x - self.x_pos_list[self.current_pose_index - 1]
            delta_y = pose_msg.pose.position.y - self.y_pos_list[self.current_pose_index - 1]
            yaw_angle = math.atan2(delta_y, delta_x)
            # Convert radians to quaternion for the orientation
            pose_msg.pose.orientation = self.calculate_quaternion_from_yaw(yaw_angle)
        else:
            # For the first pose, set a default orientation
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0

        self.publisher.publish(pose_msg)
        position_data = f'x: {pose_msg.pose.position.x}, y: {pose_msg.pose.position.y}'
        self.get_logger().info(f'Published PoseStamped message at position: {position_data}')

        self.is_goal_reached = False

    def republish_previous_pose(self):
        # Republish the previous pose if the goal hasn't been reached
        self.is_goal_reached = True

    def goal_status_callback(self, msg):
        status_list = msg.status_list
        if status_list:
            final_goal_status = status_list[-1]

            if hasattr(final_goal_status, 'status'):
                status_string = int(final_goal_status.status)

                if status_string == 4 or status_string == 6:
                    print("Goal reached")
                    self.is_goal_reached = True
                    self.current_pose_index = (self.current_pose_index + 1) % len(self.x_pos_list)

            else:
                print("The 'status' attribute is not present in the final GoalStatus object.")
        else:
            print("The status_list is empty.")

    def calculate_quaternion_from_yaw(self, yaw):
        # Convert yaw angle to quaternion
        quaternion = PoseStamped().pose.orientation
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

def main(args=None):
    rclpy.init(args=args)

    pose_publisher = PosePublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(pose_publisher, timeout_sec=0.1)

        print("Node stopped. Exiting the script.")

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Shutting down.")

    # Destroy the node explicitly
    pose_publisher.destroy_node()

    # Shutdown the ROS 2 system
    rclpy.shutdown()

if __name__ == '__main__':
    main()
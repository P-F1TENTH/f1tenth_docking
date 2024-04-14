import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from rclpy.node import Node


class MockOptitrackNode(Node):
    def __init__(self):
        super().__init__("mock_optitrack_node")
        self.publisher = self.create_publisher(
            PoseStamped, "optitrack/rigid_body_0", 10
        )
        self.timer = self.create_timer(0.1, self.publish_pose)

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "optitrack"

        # Mock position data
        pose_msg.pose.position = Point(x=0.0, y=0.0, z=1.0)

        # Mock orientation data
        pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    mock_node = MockOptitrackNode()
    rclpy.spin(mock_node)
    mock_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

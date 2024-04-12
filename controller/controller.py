import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class OptitrackSubscriber(Node):
    def __init__(self):
        super().__init__("optitrack_subscriber")
        self.subscription = self.create_subscription(
            PoseStamped,
            "optitrack/rigid_body_0",
            self.pose_callback,
            10,
        )

    def pose_callback(self, msg):
        self.get_logger().info("Received Pose: \n{}".format(msg))


def main(args=None):
    rclpy.init(args=args)
    optitrack_subscriber = OptitrackSubscriber()
    rclpy.spin(optitrack_subscriber)
    optitrack_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

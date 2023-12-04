import rclpy
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import PathWithLaneId
from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            PathWithLaneId,
            "behavior_planning/path_with_lane_id",
            self.listener_callback,
            10,
        )
        self.subscription  # prevent unused variable warning

        self.path_pub = self.create_publisher(
            PathWithLaneId,
            "behavior_planning/path_with_lane_id_modified",
            10,
        )

    def listener_callback(self, msg: PathWithLaneId):
        path = msg
        self.get_logger().info("I heard PathWithLaneId")
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

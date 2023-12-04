import rclpy
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import PathWithLandId


from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("potential_field_planner")
        self.is_start_received = False
        self.is_goal_received = False

        self.start_sub = self.create_subscription(
            PathWithLandId,
            "behavior_planning/path_with_lane_id",
            self.start_callback,
            10,
        )

        self.path_pub = self.create_publisher(
            PathWithLandId,
            "/potential_planner/path",
            10,
        )


def start_callback(self, msg: PathWithLandId):
    new_path = msg
    self.get_logger().info("I heard PathWithLaneId")
    self.path_pub.publish(new_path)


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

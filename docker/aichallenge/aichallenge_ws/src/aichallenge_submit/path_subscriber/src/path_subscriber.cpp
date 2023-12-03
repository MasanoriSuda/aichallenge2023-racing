#include "path_subscriber.hpp"
#include <memory>

using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
using std::placeholders::_1;

PathSubscriber::PathSubscriber() : Node("path_subscriber")
{
    sub_path_ = create_subscription<PathWithLaneId>("behavior_planning/path_with_lane_id", 10, std::bind(&PathSubscriber::topicCallback, this, _1));
}

void PathSubscriber::topicCallback(const PathWithLaneId::ConstSharedPtr msg)
{
    path_ = msg;
    RCLCPP_INFO(this->get_logger(), "I heard");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathSubscriber>());
    rclcpp::shutdown();
    return 0;
}

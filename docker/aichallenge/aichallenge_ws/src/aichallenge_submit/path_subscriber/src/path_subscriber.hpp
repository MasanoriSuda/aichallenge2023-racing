#ifndef PATH_SUBSCRIBER_HPP_
#define PATH_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;

class PathSubscriber : public rclcpp::Node
{
public:
    PathSubscriber();

    void topicCallback(const PathWithLaneId::ConstSharedPtr msg);

private:
    rclcpp::Subscription<PathWithLaneId>::SharedPtr sub_path_;
    PathWithLaneId::ConstSharedPtr path_;
};
#endif // PATH_SUBSCRIBER_HPP_

#ifndef PATH_SUBSCRIBER_HPP_
#define PATH_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>

using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;

class PathSubscriber : public rclcpp::Node
{
public:
    PathSubscriber(void);
    void topicCallback(const PathWithLaneId::ConstSharedPtr msg);

private:
    rclcpp::Subscription<PathWithLaneId>::ConstSharedPtr sub_path_sub_;
    rclcpp::Publisher<PathWithLaneId>::SharedPtr sub_path_pub_;
};

#endif // PATH_SUBSCRIBER_HPP_

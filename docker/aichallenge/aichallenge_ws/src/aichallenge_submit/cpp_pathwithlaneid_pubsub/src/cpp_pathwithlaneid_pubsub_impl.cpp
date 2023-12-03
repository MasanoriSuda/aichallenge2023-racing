#include "cpp_pathwithlaneid_pubsub.hpp"
#include <memory>

using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
using std::placeholders::_1;

PathSubscriber::PathSubscriber() : Node("path_subscriber")
{

  sub_path_pub_ = create_publisher<PathWithLaneId>(
      "behavior_planning/path_with_lane_id_modified", 1);

  sub_path_sub_ = create_subscription<PathWithLaneId>("behavior_planning/path_with_lane_id", 10, std::bind(&PathSubscriber::topicCallback, this, _1));
}

void PathSubscriber::topicCallback(const PathWithLaneId::ConstSharedPtr msg)
{
  PathWithLaneId path;

  path = *msg;

  /*PathPointWithLaneID*/
  path.header = msg->header;
  path.left_bound = msg->left_bound;
  path.right_bound = msg->right_bound;
  path.points = msg->points;

  /*
  path.header = msg->header;
  path.left_bound = msg->left_bound;
  path.right_bound = msg->right_bound;
  for (const auto &point : msg->points)
  {
    path.points.push_back(point.point);
  }
  */

  // RCLCPP_INFO(this->get_logger(), "I heard");

  sub_path_pub_->publish(path);
}

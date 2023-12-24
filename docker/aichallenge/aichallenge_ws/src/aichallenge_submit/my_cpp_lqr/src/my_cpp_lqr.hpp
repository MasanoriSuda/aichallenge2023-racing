#ifndef MY_PURE_PURSUIT_HPP_
#define MY_PURE_PURSUIT_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace cpprobotics
{

    using autoware_auto_control_msgs::msg::AckermannControlCommand;
    using autoware_auto_planning_msgs::msg::Trajectory;
    using autoware_auto_planning_msgs::msg::TrajectoryPoint;
    using geometry_msgs::msg::Pose;
    using geometry_msgs::msg::PoseWithCovarianceStamped;
    using geometry_msgs::msg::Twist;
    using nav_msgs::msg::Odometry;

    class MyCppLqr : public rclcpp::Node
    {
    public:
        explicit MyCppLqr();

        // subscribers
        rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
        rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
        rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_start_;

        // publishers
        rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;

        // timer
        rclcpp::TimerBase::SharedPtr timer_;

        // updated by subscribers
        Trajectory::SharedPtr trajectory_;
        Odometry::SharedPtr odometry_;
        geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
            start_;

    private:
        void onTimer();
        bool subscribeMessageAvailable();
    };
} // namespace cpprobotics

#endif // MY_PURE_PURSUIT_HPP_

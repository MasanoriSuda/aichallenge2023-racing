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

namespace my_pure_pursuit
{

    using autoware_auto_control_msgs::msg::AckermannControlCommand;
    using autoware_auto_planning_msgs::msg::Trajectory;
    using autoware_auto_planning_msgs::msg::TrajectoryPoint;
    using geometry_msgs::msg::Pose;
    using geometry_msgs::msg::PoseWithCovarianceStamped;
    using geometry_msgs::msg::Twist;
    using nav_msgs::msg::Odometry;

    class MyPurePursuit : public rclcpp::Node
    {
    public:
        explicit MyPurePursuit();

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

    class State
    {
    public:
        double x;
        double y;
        double yaw;
        double v;
        double rear_x;
        double rear_y;

        State(double x = 0.0, double y = 0.0, double yaw = 0.0, double v = 0.0);
        void update(double a, double delta, double dt);
        double calc_distance(double point_x, double point_y) const;
    };

    class States
    {
    public:
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> yaw;
        std::vector<double> v;
        std::vector<double> t;

        void append(double t, const State &state);
    };

    class TargetCourse
    {
    public:
        std::vector<double> cx;
        std::vector<double> cy;
        mutable int old_nearest_point_index;

        TargetCourse(const std::vector<double> &cx, const std::vector<double> &cy);
        std::pair<int, double> search_target_index(const State &state) const;

    private:
        double k;   // Placeholder for the value of k in Lf = k * state.v + Lfc
        double Lfc; // Placeholder for the value of Lfc in Lf = k * state.v + Lfc
    };

    State stat;
    States stats;

    double proportional_control(double target, double current);
} // namespace my_pure_pursuit

#endif // MY_PURE_PURSUIT_HPP_

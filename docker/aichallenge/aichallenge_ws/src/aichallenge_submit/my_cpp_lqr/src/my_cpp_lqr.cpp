/*************************************************************************
    > File Name: lqr_steer_control.cpp
    > Author: TAI Lei
    > Mail: ltai@ust.hk
    > Created Time: Wed Apr 17 11:48:46 2019
 ************************************************************************/

#include <iostream>
#include <limits>
#include <vector>
#include <sys/time.h>
#include <Eigen/Eigen>
#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "cubic_spline.h"
#include "motion_model.h"
#include "cpprobotics_types.h"
#include "my_cpp_lqr.hpp"

#define DT 0.03
#define L 0.19
#define KP 1.0
#define MAX_STEER 200.0 / 180 * M_PI

using namespace cpprobotics;
using Matrix5f = Eigen::Matrix<float, 5, 5>;
using Matrix52f = Eigen::Matrix<float, 5, 2>;
using Matrix25f = Eigen::Matrix<float, 2, 5>;
using RowVector5f = Eigen::Matrix<float, 1, 5>;
using Vector5f = Eigen::Matrix<float, 5, 1>;

namespace cpprobotics
{
    // ビルドエラー回避のため引数変更
    // Vec_f calc_speed_profile(Vec_f rx, Vec_f ry, Vec_f ryaw, float target_speed)
    Vec_f calc_speed_profile(Vec_f ryaw, float target_speed)
    {
        Vec_f speed_profile(ryaw.size(), target_speed);

        float direction = 1.0;
        for (unsigned int i = 0; i < ryaw.size() - 1; i++)
        {
            float dyaw = std::abs(ryaw[i + 1] - ryaw[i]);
            float switch_point = (M_PI / 4.0 < dyaw) && (dyaw < M_PI / 2.0);

            if (switch_point)
                direction = direction * -1;
            if (direction != 1.0)
                speed_profile[i] = target_speed * -1;
            else
                speed_profile[i] = target_speed;

            if (switch_point)
                speed_profile[i] = 0.0;
        }

        for (int k = 0; k < 40; k++)
        {
            *(speed_profile.end() - k) = target_speed / (50 - k);
            if (*(speed_profile.end() - k) <= 1.0 / 3.6)
            {
                *(speed_profile.end() - k) = 1.0 / 3.6;
            }
        }
        return speed_profile;
    };

    float calc_nearest_index(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, int &ind)
    {
        float mind = std::numeric_limits<float>::max();
        for (unsigned int i = 0; i < cx.size(); i++)
        {
            float idx = cx[i] - state.x;
            float idy = cy[i] - state.y;
            float d_e = idx * idx + idy * idy;

            if (d_e < mind)
            {
                mind = d_e;
                ind = i;
            }
        }
        float dxl = cx[ind] - state.x;
        float dyl = cy[ind] - state.y;
        float angle = YAW_P2P(cyaw[ind] - std::atan2(dyl, dxl));
        if (angle < 0)
            mind = mind * -1;

        return mind;
    };

    Matrix5f solve_DARE(Matrix5f A, Matrix52f B, Matrix5f Q, Eigen::Matrix2f R)
    {
        Matrix5f X = Q;
        int maxiter = 150;
        float eps = 0.01;

        for (int i = 0; i < maxiter; i++)
        {
            Matrix5f Xn = A.transpose() * X * A - A.transpose() * X * B * (R + B.transpose() * X * B).inverse() * B.transpose() * X * A + Q;
            Matrix5f error = Xn - X;
            if (error.cwiseAbs().maxCoeff() < eps)
            {
                return Xn;
            }
            X = Xn;
        }

        return X;
    };

    Matrix25f dlqr(Matrix5f A, Matrix52f B, Matrix5f Q, Eigen::Matrix2f R)
    {
        Matrix5f X = solve_DARE(A, B, Q, R);
        Matrix25f K = (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);
        return K;
    };

    Vec_f lqr_steering_control(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, Vec_f sp, float &pe, float &pth_e)
    {
        int ind = 0;
        float e = calc_nearest_index(state, cx, cy, cyaw, ind);

        float k = ck[ind];
        float th_e = YAW_P2P(state.yaw - cyaw[ind]);
        float tv = sp[ind];

        Matrix5f A = Matrix5f::Zero();
        A(0, 0) = 1.0;
        A(0, 1) = DT;
        A(1, 2) = state.v;
        A(2, 2) = 1.0;
        A(2, 3) = DT;
        A(4, 4) = 1.0;

        Matrix52f B = Matrix52f::Zero();
        B(3, 0) = state.v / L;
        B(4, 1) = DT;

        Matrix5f Q = Matrix5f::Identity();
        Eigen::Matrix2f R = Eigen::Matrix2f::Identity();

        // gain of lqr
        Matrix25f K = dlqr(A, B, Q, R);

        Vector5f x = Vector5f::Zero();
        x(0) = e;
        x(1) = (e - pe) / DT;
        x(2) = th_e;
        x(3) = (th_e - pth_e) / DT;
        x(4) = state.v - tv;

        Eigen::Vector2f ustar = -K * x;

        float ff = std::atan2((L * k), (double)1.0);
        float fb = YAW_P2P(ustar(0));
        float delta = ff + fb;
        float ai = ustar(1);

        pe = e;
        pth_e = th_e;
        return {ai, delta};
    };

    void update(State &state, float a, float delta)
    {

        if (delta >= MAX_STEER)
            delta = MAX_STEER;
        if (delta <= -MAX_STEER)
            delta = -MAX_STEER;

        state.x = state.x + state.v * std::cos(state.yaw) * DT;
        state.y = state.y + state.v * std::sin(state.yaw) * DT;
        state.yaw = state.yaw + state.v / L * std::tan(delta) * DT;
        state.v = state.v + a * DT;
    };

    State state(-0.0, -0.0, 0.0, 0.0);
    void closed_loop_prediction(Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, Vec_f speed_profile)
    {
        // ビルドエラー回避
        // float stop_speed = 0.05;

        Vec_f x;
        x.push_back(state.x);
        Vec_f y;
        y.push_back(state.y);
        Vec_f yaw;
        yaw.push_back(state.yaw);
        Vec_f v;
        v.push_back(state.v);
        Vec_f t;
        t.push_back(0.0);

        float e = 0;
        float e_th = 0;

        // ビルドエラー回避
        // int count = 0;
        Vec_f x_h;
        Vec_f y_h;

        Vec_f control = lqr_steering_control(state, cx, cy, cyaw, ck, speed_profile, e, e_th);
        // float ai = KP * (speed_profile[ind]-state.v);
        update(state, control[0], control[1]);
        // if (std::abs(state.v) <= stop_speed) ind += 1;
    };

    MyCppLqr::MyCppLqr() : Node("my_pure_pursuit")
    {

        pub_cmd_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", 1);

        sub_kinematics_ = create_subscription<Odometry>(
            "/localization/kinematic_state", 1, [this](const Odometry::SharedPtr msg)
            { odometry_ = msg; });
        sub_trajectory_ = create_subscription<Trajectory>(
            "/planning/scenario_planning/trajectory", 1, [this](const Trajectory::SharedPtr msg)
            { trajectory_ = msg; });

        using namespace std::literals::chrono_literals;
        timer_ =
            rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&MyCppLqr::onTimer, this));
    }

    AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
    {
        AckermannControlCommand cmd;
        cmd.stamp = stamp;
        cmd.longitudinal.stamp = stamp;
        cmd.longitudinal.speed = 0.0;
        cmd.longitudinal.acceleration = 0.0;
        cmd.lateral.stamp = stamp;
        cmd.lateral.steering_tire_angle = 0.0;
        return cmd;
    }

    bool MyCppLqr::subscribeMessageAvailable()
    {
        if (!odometry_)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
            return false;
        }
        if (!trajectory_)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
            return false;
        }
        return true;
    }

    void MyCppLqr::onTimer()
    {
        RCLCPP_INFO(this->get_logger(), "start onTimer()");
        // check data
        if (!subscribeMessageAvailable())
        {
            RCLCPP_INFO(this->get_logger(), "subscribeMessageAvailable not found");
            return;
        }

        // publish zero command
        AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

        Vec_f wx;
        Vec_f wy;

        for (long unsigned int i = 0; i < trajectory_->points.size(); ++i)
        {
            wx.push_back(trajectory_->points[i].pose.position.x);
            wy.push_back(trajectory_->points[i].pose.position.y);
        }

        Spline2D csp_obj(wx, wy);
        Vec_f r_x;
        Vec_f r_y;
        Vec_f ryaw;
        Vec_f rcurvature;
        Vec_f rs;
        for (float i = 0; i < csp_obj.s.back(); i++)
        {
            std::array<float, 2> point_ = csp_obj.calc_postion(i);
            r_x.push_back(point_[0]);
            r_y.push_back(point_[1]);
            ryaw.push_back(csp_obj.calc_yaw(i));
            rcurvature.push_back(csp_obj.calc_curvature(i));
            rs.push_back(i);
        }
        // float target_speed = 10.0 / 3.6;
        //  ビルドエラー回避
        //  Vec_f speed_profile = calc_speed_profile(r_x, r_y, ryaw, target_speed);
        Vec_f speed_profile;
        for (float i = 0; i < csp_obj.s.back(); i++)
        {
            if (i < trajectory_->points.size())
            {
                speed_profile.push_back(trajectory_->points.at(i).longitudinal_velocity_mps);
            }
            else
            {
                speed_profile.push_back(trajectory_->points.at(trajectory_->points.size() - 1).longitudinal_velocity_mps);
            }
        }

        state.x = odometry_->pose.pose.position.x;
        state.y = odometry_->pose.pose.position.y;
        state.yaw = tf2::getYaw(odometry_->pose.pose.orientation);
        state.v = odometry_->twist.twist.linear.x;
        closed_loop_prediction(r_x, r_y, ryaw, rcurvature, speed_profile);

        cmd.longitudinal.speed = state.v;
        cmd.longitudinal.acceleration = state.v - odometry_->twist.twist.linear.x;
        if (cmd.longitudinal.acceleration > 1)
        {
            // cmd.longitudinal.acceleration /= 2;
        }
        if (cmd.longitudinal.acceleration < 0)
        {
            cmd.longitudinal.acceleration *= 500;
        }
        cmd.lateral.steering_tire_angle = state.yaw - tf2::getYaw(odometry_->pose.pose.orientation);

        RCLCPP_INFO(this->get_logger(), "Sending Cmd");
        pub_cmd_->publish(cmd);
    }
} // my_cpp_lqr

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cpprobotics::MyCppLqr>());
    rclcpp::shutdown();
    return 0;
}

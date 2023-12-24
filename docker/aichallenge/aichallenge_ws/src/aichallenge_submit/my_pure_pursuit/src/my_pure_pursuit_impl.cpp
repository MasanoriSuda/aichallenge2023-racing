#include "my_pure_pursuit.hpp"
#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <vector>

float k = 0.44;  /*look forward gain*/
float Lfc = 5.0; /*[m] look-ahead distance*/
float Kp = 0.5;  /*speed proportional gain*/
float dt = 0.03; /*[s] time tick*/
float WB = 3.0;  /*[m] wheel base of vehicle*/

bool isinitialized = false;

namespace my_pure_pursuit
{
  using std::placeholders::_1;

  MyPurePursuit::MyPurePursuit() : Node("my_pure_pursuit")
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
        rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&MyPurePursuit::onTimer, this));

    stat = State(0, 0, 0, 0);
    stats = States();
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

  double proportional_control(double target, double current)
  {
    return Kp * (target - current);
  }

  std::pair<int, double> pure_pursuit_steer_control(const State &state, const TargetCourse &trajectory, int pind)
  {
    int ind;
    double Lf;
    std::tie(ind, Lf) = trajectory.search_target_index(state);

    if (pind >= ind)
    {
      ind = pind;
    }

    double tx, ty;

    if ((long unsigned int)ind < trajectory.cx.size())
    {
      tx = trajectory.cx[ind];
      ty = trajectory.cy[ind];
    }
    else
    { // toward goal
      tx = trajectory.cx.back();
      ty = trajectory.cy.back();
      ind = trajectory.cx.size() - 1;
    }

    double alpha = std::atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw;

    double delta = std::atan2(2.0 * WB * std::sin(alpha) / Lf, 1.0);

    std::pair<int, double> result = {ind, delta};

    return result;
  }

  bool isInitialized = false;
  std::vector<double> tmp;
  // TargetCourse target_course(tmp, tmp);
  int target_ind = 0;
  void MyPurePursuit::onTimer()
  {
    RCLCPP_INFO(this->get_logger(), "start onTimer()");
    // check data
    if (!subscribeMessageAvailable())
    {
      RCLCPP_INFO(this->get_logger(), "subscribeMessageAvailable not found");
      return;
    }

    size_t closet_traj_point_idx =
        motion_utils::findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

    // publish zero command
    AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

    if (
        (closet_traj_point_idx == trajectory_->points.size() - 1) ||
        (trajectory_->points.size() <= 5))
    {
      cmd.longitudinal.speed = 0.0;
      cmd.longitudinal.acceleration = -10.0;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
    }
    else
    {

      // calc longitudinal speed and acceleration
      double current_longitudinal_vel = odometry_->twist.twist.linear.x;

      // target course
      int startx, starty;
      startx = odometry_->pose.pose.position.x;
      starty = odometry_->pose.pose.position.y;
      std::vector<double> cx;
      std::vector<double> cy;
      RCLCPP_INFO(this->get_logger(), "trajectory size = %ld", trajectory_->points.size());
      for (long unsigned int i = 0; i < trajectory_->points.size(); ++i)
      {
        int startx, starty;
        startx = trajectory_->points[i].pose.position.x;
        starty = trajectory_->points[i].pose.position.y;
        cx.push_back((double)startx);
        cy.push_back((double)starty);
      }

      TargetCourse target_course(cx, cy);
      // target_course.cx = cx;
      // target_course.cy = cy;

      stat = State(startx, starty, tf2::getYaw(odometry_->pose.pose.orientation), current_longitudinal_vel);
      target_ind = target_course.search_target_index(stat).first;

      RCLCPP_INFO(this->get_logger(), "yaw=%f", tf2::getYaw(odometry_->pose.pose.orientation));

      // get closest trajectory point from current position

      // RCLCPP_INFO(this->get_logger(), "I heard in onTimer() %d", target_ind);

      double ai = proportional_control(trajectory_->points.at(target_ind).longitudinal_velocity_mps, current_longitudinal_vel);
      std::pair<double, int> result = pure_pursuit_steer_control(stat, target_course, target_ind);
      target_ind = result.first;
      double di = result.second;

      RCLCPP_INFO(this->get_logger(), "I heard in onTimer() %d", target_ind);

      stat.update(ai, di, 1); // Control vehicle
      stat.v = current_longitudinal_vel;
      // cmd.lateral.steering_tire_angle = target_ind;
      RCLCPP_INFO(this->get_logger(), "Sending cmd");
      cmd.stamp = get_clock()->now();
      cmd.longitudinal.stamp = get_clock()->now();
      //      cmd.longitudinal.speed = stat.v;
      //      cmd.longitudinal.acceleration = Kp * (stat.v - current_longitudinal_vel);
      cmd.longitudinal.speed = trajectory_->points.at(target_ind).longitudinal_velocity_mps;
      cmd.longitudinal.acceleration = trajectory_->points.at(target_ind).longitudinal_velocity_mps - current_longitudinal_vel;
      // cmd.longitudinal.speed = 100;
      // cmd.longitudinal.acceleration = 100;
      cmd.lateral.stamp = get_clock()->now();
      cmd.lateral.steering_tire_angle = di * M_PI / 180.0;
      ;
      RCLCPP_INFO(this->get_logger(), "di=%f", di);
      // cmd.lateral.steering_tire_angle = 0;

      pub_cmd_->publish(cmd);
    }
  }
  bool MyPurePursuit::subscribeMessageAvailable()
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

  State::State(double x, double y, double yaw, double v) : x(x), y(y), yaw(yaw), v(v)
  {
    rear_x = x - ((WB / 2) * cos(yaw));
    rear_y = y - ((WB / 2) * sin(yaw));
  }

  void State::update(double a, double delta, double dt)
  {
    x = v * cos(yaw) * dt;
    y = v * sin(yaw) * dt;
    yaw = (v / WB) * tan(delta) * dt;
    v = a * dt;
    rear_x = x - ((WB / 2) * cos(yaw));
    rear_y = y - ((WB / 2) * sin(yaw));
  }

  double State::calc_distance(double point_x, double point_y) const
  {
    double dx = rear_x - point_x;
    double dy = rear_y - point_y;
    return hypot(dx, dy);
  }

  void States::append(double t, const State &state)
  {
    x.push_back(state.x);
    y.push_back(state.y);
    yaw.push_back(state.yaw);
    v.push_back(state.v);
    this->t.push_back(t);
  }

  TargetCourse::TargetCourse(const std::vector<double> &cx, const std::vector<double> &cy)
      : cx(cx), cy(cy), old_nearest_point_index(-1), k(0.1), Lfc(2.0) {} // Placeholder values for k and Lfc

  std::pair<int, double> TargetCourse::search_target_index(const State &state) const
  {
    int ind;

    // To speed up nearest point search, doing it only at the first time.
    if (old_nearest_point_index == -1)
    {
      // Search nearest point index

      std::vector<double> dx(cx.size()), dy(cy.size());
      std::transform(cx.begin(), cx.end(), dx.begin(), [state](double icx)
                     { return state.rear_x - icx; });
      std::transform(cy.begin(), cy.end(), dy.begin(), [state](double icy)
                     { return state.rear_y - icy; });

      std::vector<double> d(cx.size());
      std::transform(dx.begin(), dx.end(), dy.begin(), d.begin(), [](double x, double y)
                     { return hypot(x, y); });

      auto min_d = std::min_element(d.begin(), d.end());
      ind = std::distance(d.begin(), min_d);
      old_nearest_point_index = ind;
    }
    else
    {
      ind = old_nearest_point_index;
      double distance_this_index = state.calc_distance(cx[ind], cy[ind]);

      while (true)
      {
        double distance_next_index = state.calc_distance(cx[ind + 1], cy[ind + 1]);
        if (distance_this_index < distance_next_index)
        {
          break;
        }
        ind = ((long unsigned int)(ind + 1) < cx.size()) ? ind + 1 : ind;
        distance_this_index = distance_next_index;
      }

      old_nearest_point_index = ind;
    }

    double Lf = k * state.v + Lfc; // Update look ahead distance

    // Search look ahead target point index
    while (Lf > state.calc_distance(cx[ind], cy[ind]))
    {
      if ((long unsigned int)(ind + 1) >= cx.size())
      {
        break; // Do not exceed the goal
      }
      ++ind;
    }

    return std::make_pair(ind, Lf);
  }
} // namespace my_pure_pursuit

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_pure_pursuit::MyPurePursuit>());
  rclcpp::shutdown();
  return 0;
}

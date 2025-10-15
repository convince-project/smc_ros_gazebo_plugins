/*
 * Copyright (c) 2025 - for information on the respective copyright owner
 * see the NOTICE file
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <roamer_pkg/action/roamer.hpp>  // generated from action/Roamer.action

// Class that defines a ROS2 action server to drive a robot forward
// It monitors laser scans for obstacles and odometry for pose
class RoamerServer : public rclcpp::Node
{
public:
  using ActionT    = roamer_pkg::action::Roamer;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using Goal       = ActionT::Goal;
  using Result     = ActionT::Result;
  using Feedback   = ActionT::Feedback;

  // Constructor sets up subscriptions, publishers, parameters, and action server
  explicit RoamerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("roamer_server", options)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    // Declare parameters for configuration
    action_name_        = "roamer";
    bump_threshold_     = 0.2;
    feedback_rate_hz_   = 100.0;
    cmd_vel_linear_     = 0.1;
    cmd_vel_angular_    = 0.2;

    // Subscribe to laser scans for collision detection
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&RoamerServer::on_scan, this, _1));

    // Subscribe to odometry for pose updates
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SystemDefaultsQoS(),
      std::bind(&RoamerServer::on_odom, this, _1));

    // Publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS());

    // Create the action server
    server_ = rclcpp_action::create_server<ActionT>(
      this,
      action_name_,
      std::bind(&RoamerServer::handle_goal,     this, _1, _2),
      std::bind(&RoamerServer::handle_cancel,   this, _1),
      std::bind(&RoamerServer::handle_accepted, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "RoamerServer ready on action '%s'", action_name_.c_str());
  }

private:
  // Callback for laser scan messages, detects if the robot is too close to an obstacle
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    bool bumped_local = false;
    for (float r : msg->ranges) {
      if (std::isfinite(r) && r <= bump_threshold_) { bumped_local = true; break; }
    }
    bumped_.store(bumped_local, std::memory_order_relaxed);
  }

  // Callback for odometry messages, stores the latest robot pose
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    last_pose_ = msg->pose.pose;
    have_pose_ = true;
  }

  // Handle new goal requests, always accept and execute
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const Goal> /*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle cancellation requests, always accept
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // When a goal is accepted, start execution in a new detached thread
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&RoamerServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // Execution loop for the action goal
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {

    
    auto goal = goal_handle->get_goal();
    const double target_d = goal->distance_m;   // meters
    const double target_a = goal->angle_rad;    // radians, CCW

    rclcpp::Rate loop_rate(feedback_rate_hz_, this->get_clock());
    auto feedback = std::make_shared<Feedback>();
    auto result   = std::make_shared<Result>();

    // helpers
    auto quat_to_yaw = [](const geometry_msgs::msg::Quaternion &q) {
      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      return std::atan2(siny_cosp, cosy_cosp);
    };
    auto ang_norm = [](double a){
      while (a >  M_PI) a -= 2.0*M_PI;
      while (a < -M_PI) a += 2.0*M_PI;
      return a;
    };

    // wait for first pose
    while (rclcpp::ok() && !have_pose_) {
      if (goal_handle->is_canceling()) { goal_handle->canceled(result); return; }
      loop_rate.sleep();
    }

    // snapshot start pose/yaw (Pose, not PoseStamped)
    geometry_msgs::msg::Pose start_pose;
    {
      std::lock_guard<std::mutex> lk(pose_mtx_);
      start_pose = last_pose_;
    }
    const double start_x   = start_pose.position.x;
    const double start_y   = start_pose.position.y;
    const double start_yaw = quat_to_yaw(start_pose.orientation);

    // tolerances
    const double ang_tol = 0.1;  // ~5.73 deg
    const double lin_tol = 0.1;  // 10 cm

    // rotate then translate
    bool rotate_done    = std::abs(ang_norm(target_a)) < ang_tol;
    bool translate_done = (target_d <= lin_tol);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stop_robot();
        { std::lock_guard<std::mutex> lk(pose_mtx_); result->pose = last_pose_; }
        result->bumped = false;
        goal_handle->canceled(result);
        return;
      }

      // speeds (params override defaults)
      double v_lin = cmd_vel_linear_;
      double v_ang = cmd_vel_angular_;

      // current pose snapshot
      geometry_msgs::msg::Pose cur_pose;
      {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        cur_pose = last_pose_;
        feedback->pose = last_pose_;     // Feedback.pose is a Pose
        feedback->bumped = false;
        feedback->finished = false;
      }
      goal_handle->publish_feedback(feedback);

      // errors
      const double cur_x   = cur_pose.position.x;
      const double cur_y   = cur_pose.position.y;
      const double cur_yaw = quat_to_yaw(cur_pose.orientation);

      const double yaw_err = ang_norm((start_yaw + target_a) - cur_yaw);
      const double dx = cur_x - start_x;
      const double dy = cur_y - start_y;
      const double dist_traveled = std::hypot(dx, dy);
      const double dist_err = target_d - dist_traveled;

      geometry_msgs::msg::Twist cmd{};

      if (!rotate_done) {
        if (std::abs(yaw_err) <= ang_tol) {
          rotate_done = true;
        } else {
          cmd.angular.z = (yaw_err > 0.0 ? +v_ang : -v_ang);
          cmd.linear.x  = 0.0;
        }
      } else if (!translate_done) {
        if (dist_err <= lin_tol) {
          translate_done = true;
        } else {
          cmd.linear.x  = std::clamp(dist_err, -v_lin, v_lin);
          cmd.angular.z = 0.0;
        }
      } else {
        stop_robot();
        { std::lock_guard<std::mutex> lk(pose_mtx_); result->pose = last_pose_; }
        result->bumped = false;
        result->finished = true;
        goal_handle->succeed(result);
        return;
      }

      cmd_vel_pub_->publish(cmd);

      if (bumped_.load(std::memory_order_relaxed)) {
        stop_robot();
        std::lock_guard<std::mutex> lk(pose_mtx_);
        result->pose   = last_pose_;
        result->bumped = true;
        result->finished = true;
        goal_handle->succeed(result);
        return;
      }

      loop_rate.sleep();
    }

    stop_robot();
    { std::lock_guard<std::mutex> lk(pose_mtx_); result->pose = last_pose_; }
    result->bumped = false;
    goal_handle->abort(result);
  }


  // Helper function to stop the robot by sending zero velocity
  void stop_robot()
  {
    geometry_msgs::msg::Twist zero{};
    zero.linear.x = 0.0;
    zero.linear.y = 0.0;
    zero.linear.z = 0.0;
    zero.angular.x = 0.0;
    zero.angular.y = 0.0;
    zero.angular.z = 0.0;
    cmd_vel_pub_->publish(zero);
  }

private:
  // Action server
  rclcpp_action::Server<ActionT>::SharedPtr server_;

  // Subscriptions and publisher
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub_;

  // State variables
  std::mutex pose_mtx_;
  geometry_msgs::msg::Pose last_pose_{};
  std::atomic<bool> bumped_{false};
  bool have_pose_{false};

  // Parameters
  std::string action_name_;
  double bump_threshold_;
  double feedback_rate_hz_;
  double cmd_vel_linear_{0.2};
  double cmd_vel_angular_{0.2};
};

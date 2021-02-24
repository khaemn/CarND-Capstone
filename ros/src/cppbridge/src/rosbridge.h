#pragma once
#include "json.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <set>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <vector>
#include <uWS/uWS.h>
#include <memory>

static constexpr auto INITIAL_YAW = 360;

/// Implements a bridge between uWS and ROS topics
class RosBridge
{
public:
  explicit RosBridge(std::weak_ptr<uWS::Hub> hub, int argc, char **argv);

  void handle_telemetry(const nlohmann::json& data);

  std_msgs::Bool to_dbw_status(const nlohmann::json &data);

  geometry_msgs::PoseStamped parse_pose(const nlohmann::json &data);

  geometry_msgs::TwistStamped parse_velocities(const nlohmann::json &data);

private:
  double compute_angular_velocity(double new_yaw);

private:
  std::weak_ptr<uWS::Hub> hub_;

  ros::Publisher dbw_status_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;

  ros::Time prev_time_;

  bool is_dbw_enabled_ = true;
  double prev_yaw_ = INITIAL_YAW;
  double prev_angular_vel_ = 0.0;
  const double angular_velocity_filter_coeff_ = 10;
};

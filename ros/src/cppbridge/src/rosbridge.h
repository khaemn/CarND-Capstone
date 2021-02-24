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


/// Implements a bridge between uWS and ROS topics
class RosBridge
{
public:
  explicit RosBridge(std::weak_ptr<uWS::Hub> hub, int argc, char **argv);

  void handle_telemetry(const nlohmann::json& data);

  static std_msgs::Bool to_dbw_status(const nlohmann::json &data);

  static geometry_msgs::PoseStamped to_pose(const nlohmann::json &data);

  static geometry_msgs::TwistStamped to_velocity(const nlohmann::json &data);

private:
  std::weak_ptr<uWS::Hub> hub_;

  ros::Publisher dbw_status_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;

  bool is_dbw_enabled_ = true;
  double prev_heading_angle_= 361.;
};

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
#include <styx_msgs/TrafficLight.h>
#include <styx_msgs/Lane.h>

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

  void publish_changed_dbw_status(const nlohmann::json &data);

  geometry_msgs::PoseStamped parse_position(const nlohmann::json &data) const;

  geometry_msgs::TwistStamped parse_velocities(const nlohmann::json &data);

  void handle_traffic_lights(const nlohmann::json &data);

  std::string get_waypoints_tcp_message() const;

  float steering_angle() const;
  float throttle_val() const;
  float brake_val() const;

private:
  double compute_angular_velocity(double new_yaw);
  geometry_msgs::PoseStamped create_pose(double x, double y, double z, double yaw_degree) const;
  styx_msgs::TrafficLight    create_light(double x, double y, double z, double yaw_degree,
                                          uint8_t state) const;
  void                       final_wpts_callback(const styx_msgs::Lane &lane);

private:
  std::weak_ptr<uWS::Hub> hub_;

  ros::Publisher dbw_status_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher trafficlight_pub_;

  ros::Subscriber final_wpts_sub_;
  ros::Subscriber steering_sub_;
  ros::Subscriber throttle_sub_;
  ros::Subscriber brake_sub_;

  ros::Time prev_time_;

  bool is_dbw_enabled_ = true;
  double prev_yaw_ = INITIAL_YAW;
  double prev_angular_vel_ = 0.0;
  const double angular_velocity_filter_coeff_ = 10;

  // Storage for the waypoint coords to send to the simulator
  std::vector<double> wpt_xs_;
  std::vector<double> wpt_ys_;
  std::vector<double> wpt_zs_;
};

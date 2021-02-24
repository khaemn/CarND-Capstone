#include "rosbridge.h"

#include <ros/time.h>
#include <ros/duration.h>
#include <tf/transform_datatypes.h>


static constexpr auto PUBNAME_DBW_STATUS = "/vehicle/dbw_enabled";
static constexpr auto PUBNAME_VELOCITY   = "/current_velocity";
static constexpr auto PUBNAME_POSE       = "/current_pose";

static constexpr auto MSG_QUEUE_SIZE = 20;

RosBridge::RosBridge(std::weak_ptr<uWS::Hub> hub, int argc, char **argv)
  : hub_(hub)
{
  ros::init(argc, argv, "simulator_bridge");
  ros::NodeHandle n;

  dbw_status_pub_ = n.advertise<std_msgs::Bool>(PUBNAME_DBW_STATUS, MSG_QUEUE_SIZE);
  pose_pub_       = n.advertise<geometry_msgs::PoseStamped>(PUBNAME_POSE, MSG_QUEUE_SIZE);
  velocity_pub_   = n.advertise<geometry_msgs::TwistStamped>(PUBNAME_VELOCITY, MSG_QUEUE_SIZE);
}

void RosBridge::handle_telemetry(const nlohmann::json &data)
{
  auto dbw_msg = to_dbw_status(data);
  dbw_status_pub_.publish(dbw_msg);

  auto pose_msg = parse_pose(data);
  pose_pub_.publish(pose_msg);

  auto velocity_msg = parse_velocities(data);
  velocity_pub_.publish(velocity_msg);
}

std_msgs::Bool RosBridge::to_dbw_status(const nlohmann::json &data)
{
  const bool     enabled = data["dbw_enable"];
  std_msgs::Bool msg;
  msg.data = enabled;
  return msg;
}

geometry_msgs::PoseStamped RosBridge::parse_pose(const nlohmann::json &data)
{
  geometry_msgs::PoseStamped msg;
  std_msgs::Header           hdr;
  hdr.stamp    = ros::Time::now();
  hdr.frame_id = "/world";
  msg.header   = hdr;

  msg.pose.position.x = data["x"];
  msg.pose.position.y = data["y"];
  msg.pose.position.z = data["z"];

  static constexpr auto euler_roll  = 0.0;
  static constexpr auto euler_pitch = 0.0;
  const auto euler_yaw   = M_PI * double(data["yaw"]) / 180.;

  auto quaternion = tf::createQuaternionFromRPY(euler_roll, euler_pitch, euler_yaw);

  geometry_msgs::Quaternion orientation;
  orientation.x = quaternion.x();
  orientation.y = quaternion.y();
  orientation.z = quaternion.z();
  orientation.w = quaternion.w();

  msg.pose.orientation = orientation;

  return msg;
}

geometry_msgs::TwistStamped RosBridge::parse_velocities(const nlohmann::json &data)
{
  geometry_msgs::TwistStamped velocity;

  // Velocity is measured in the vehicle's CS, so the linear velocity is
  // always along 'x', and the angular one is around 'z' axes.
  velocity.twist.linear.x  = double(data["velocity"]) * 0.44704;  // MPH to m/s
  velocity.twist.angular.z = compute_angular_velocity(double(data["yaw"]));
  return velocity;
}

double RosBridge::compute_angular_velocity(double new_yaw)
{
  if (prev_yaw_ >= INITIAL_YAW)
  {
    prev_yaw_  = new_yaw;
    prev_time_ = ros::Time::now();
    prev_angular_vel_ = 0.0;
    return 0;
  }

  const ros::Duration elapsed = ros::Time::now() - prev_time_;

  const auto angular_vel = (new_yaw - prev_yaw_) / (elapsed.toSec());
  const auto filtered_ang_vel =
      (prev_angular_vel_ * (angular_velocity_filter_coeff_ - 1) + angular_vel) /
      angular_velocity_filter_coeff_;
  prev_yaw_  = new_yaw;
  prev_time_ = ros::Time::now();
  prev_angular_vel_ = filtered_ang_vel;
  return filtered_ang_vel;
}

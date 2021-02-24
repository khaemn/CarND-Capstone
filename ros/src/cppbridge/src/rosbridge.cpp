#include "rosbridge.h"

#include <ros/duration.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>

static constexpr auto PUBNAME_DBW_STATUS = "/vehicle/dbw_enabled";
static constexpr auto PUBNAME_VELOCITY   = "/current_velocity";
static constexpr auto PUBNAME_POSE       = "/current_pose";

static constexpr auto SUBNAME_WAYPOINTS = "/final_waypoints";

static constexpr auto MSG_QUEUE_SIZE = 20;

static styx_msgs::Lane
    lane_buf;  // Workaround! BUt better that passing of member method as a callback.

void wpts_callback(styx_msgs::Lane lane)
{
  ROS_INFO("AZAZA");
  lane_buf = lane;
}

RosBridge::RosBridge(std::weak_ptr<uWS::Hub> hub, int argc, char **argv)
  : hub_(hub)
{
  ros::init(argc, argv, "simulator_bridge");
  ros::NodeHandle n;

  dbw_status_pub_ = n.advertise<std_msgs::Bool>(PUBNAME_DBW_STATUS, MSG_QUEUE_SIZE);
  pose_pub_       = n.advertise<geometry_msgs::PoseStamped>(PUBNAME_POSE, MSG_QUEUE_SIZE);
  velocity_pub_   = n.advertise<geometry_msgs::TwistStamped>(PUBNAME_VELOCITY, MSG_QUEUE_SIZE);

  // NOTE: I can not pass the a callback that points to a non-static member function.
  // Can solve via global variables, not the best thing either...
  final_wpts_sub_ = n.subscribe<styx_msgs::Lane>(
      SUBNAME_WAYPOINTS, MSG_QUEUE_SIZE, wpts_callback);
  ros::spinOnce();
}

void RosBridge::handle_telemetry(const nlohmann::json &data)
{
  // TODO: impl. only a 'delta'-publishing.
  auto dbw_msg = to_dbw_status(data);
  dbw_status_pub_.publish(dbw_msg);

  auto pose_msg = parse_position(data);
  pose_pub_.publish(pose_msg);

  auto velocity_msg = parse_velocities(data);
  velocity_pub_.publish(velocity_msg);

  // TODO: impl. call to emitting of the steer commands?

  ros::spinOnce();

  final_wpts_callback(lane_buf);
}

std_msgs::Bool RosBridge::to_dbw_status(const nlohmann::json &data)
{
  const bool     enabled = data["dbw_enable"];
  std_msgs::Bool msg;
  msg.data = enabled;
  return msg;
}

geometry_msgs::PoseStamped RosBridge::parse_position(const nlohmann::json &data)
{
  const double x          = data["x"];
  const double y          = data["y"];
  const double z          = data["z"];
  const double yaw_degree = data["yaw"];

  return create_pose(x, y, z, yaw_degree);
}

geometry_msgs::TwistStamped RosBridge::parse_velocities(const nlohmann::json &data)
{
  geometry_msgs::TwistStamped velocity;

  // Velocity is measured in the vehicle's CS, so the linear velocity is
  // always along 'x', and the angular one is around 'z' axes.
  // The 'y' is headed to the left, 'z' - up.
  velocity.twist.linear.x  = double(data["velocity"]) * 0.44704;  // MPH to m/s
  velocity.twist.angular.z = compute_angular_velocity(double(data["yaw"]));
  return velocity;
}

std::string RosBridge::get_waypoints_tcp_message() const
{
  nlohmann::json msgJson;
  msgJson["next_x"]     = wpt_xs_;
  msgJson["next_y"]     = wpt_ys_;
  msgJson["next_z"]     = wpt_zs_;
  const std::string msg = "42[\"drawline\"," + msgJson.dump() + "]";
  return msg;
}

double RosBridge::compute_angular_velocity(double new_yaw)
{
  if (prev_yaw_ >= INITIAL_YAW)
  {
    prev_yaw_         = new_yaw;
    prev_time_        = ros::Time::now();
    prev_angular_vel_ = 0.0;
    return 0;
  }

  const ros::Duration elapsed = ros::Time::now() - prev_time_;

  const auto angular_vel = (new_yaw - prev_yaw_) / (elapsed.toSec());
  const auto filtered_ang_vel =
      (prev_angular_vel_ * (angular_velocity_filter_coeff_ - 1) + angular_vel) /
      angular_velocity_filter_coeff_;
  prev_yaw_         = new_yaw;
  prev_time_        = ros::Time::now();
  prev_angular_vel_ = filtered_ang_vel;
  return filtered_ang_vel;
}

geometry_msgs::PoseStamped RosBridge::create_pose(double x, double y, double z,
                                                  double yaw_degree) const
{
  geometry_msgs::PoseStamped pose_stamped;
  std_msgs::Header           hdr;
  hdr.stamp           = ros::Time::now();
  hdr.frame_id        = "/world";
  pose_stamped.header = hdr;

  pose_stamped.pose.position.x = x;
  pose_stamped.pose.position.y = y;
  pose_stamped.pose.position.z = z;

  static constexpr auto euler_roll  = 0.0;
  static constexpr auto euler_pitch = 0.0;
  const auto            euler_yaw   = M_PI * yaw_degree / 180.;

  auto quaternion = tf::createQuaternionFromRPY(euler_roll, euler_pitch, euler_yaw);

  geometry_msgs::Quaternion orientation;
  orientation.x = quaternion.x();
  orientation.y = quaternion.y();
  orientation.z = quaternion.z();
  orientation.w = quaternion.w();

  pose_stamped.pose.orientation = orientation;

  return pose_stamped;
}

void RosBridge::final_wpts_callback(const styx_msgs::Lane &lane)
{
  wpt_xs_.clear();
  wpt_ys_.clear();
  wpt_zs_.clear();
  for (const auto &wpt : lane.waypoints)
  {
    wpt_xs_.push_back(wpt.pose.pose.position.x);
    wpt_ys_.push_back(wpt.pose.pose.position.y);
    wpt_zs_.push_back(wpt.pose.pose.position.z + 0.5);
  }
}

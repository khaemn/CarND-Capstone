#include "rosbridge.h"

#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <sensor_msgs/Image.h>
#include <styx_msgs/TrafficLightArray.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>

#include "cpp-base64/base64.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgcodecs.hpp>

using std::vector;
using std::cout;
using std::endl;
/*
{'topic': '/current_pose', 'type': 'pose', 'name': 'current_pose'},
{'topic': '/current_velocity', 'type': 'twist', 'name': 'current_velocity'},
 {'topic': '/vehicle/steering_report', 'type': 'steer', 'name': 'steering_report'},
 {'topic': '/vehicle/throttle_report', 'type': 'float', 'name': 'throttle_report'},
 {'topic': '/vehicle/brake_report', 'type': 'float', 'name': 'brake_report'},
 {'topic': '/vehicle/obstacle', 'type': 'pose', 'name': 'obstacle'},
 {'topic': '/vehicle/obstacle_points', 'type': 'pcl', 'name': 'obstacle_points'},
 {'topic': '/vehicle/lidar', 'type': 'pcl', 'name': 'lidar'},
{'topic': '/vehicle/traffic_lights', 'type': 'trafficlights', 'name': 'trafficlights'},
{'topic': '/vehicle/dbw_enabled', 'type': 'bool', 'name': 'dbw_status'},
 {'topic': '/image_color', 'type': 'image', 'name': 'image'},
*/
static constexpr auto PUBNAME_POSE     = "/current_pose";
static constexpr auto PUBNAME_VELOCITY = "/current_velocity";
// ...
static constexpr auto PUBNAME_TRAFFIC_LIGHTS = "/vehicle/traffic_lights";
static constexpr auto PUBNAME_DBW_STATUS     = "/vehicle/dbw_enabled";
static constexpr auto PUBNAME_IMAGE          = "/image_color";
/*
{'topic':'/vehicle/steering_cmd', 'type': 'steer_cmd', 'name': 'steering'},
{'topic':'/vehicle/throttle_cmd', 'type': 'throttle_cmd', 'name': 'throttle'},
{'topic':'/vehicle/brake_cmd', 'type': 'brake_cmd', 'name': 'brake'},
{'topic':'/final_waypoints', 'type': 'path_draw', 'name': 'path'},
*/
static constexpr auto SUBNAME_STEERING  = "/vehicle/steering_cmd";  // dbw_mkz_msgs/SteeringCmd
static constexpr auto SUBNAME_THROTTLE  = "/vehicle/throttle_cmd";  // dbw_mkz_msgs/ThrottleCmd
static constexpr auto SUBNAME_BRAKE     = "/vehicle/brake_cmd";     // dbw_mkz_msgs/BrakeCmd
static constexpr auto SUBNAME_WAYPOINTS = "/final_waypoints";       // styx_msgs/Lane

static constexpr auto MSG_QUEUE_SIZE = 5;

// Workarounds for storing values from ROS subscriber callbacks
// But it's better than passing of member method as a callback.
static styx_msgs::Lane lane_buf;

void wpts_callback(const styx_msgs::Lane &lane)
{
  lane_buf = lane;
}

static dbw_mkz_msgs::SteeringCmd steering_cmd;

void steering_callback(const dbw_mkz_msgs::SteeringCmd &cmd)
{
  steering_cmd = cmd;
}

static dbw_mkz_msgs::BrakeCmd brake_cmd;

void brake_callback(const dbw_mkz_msgs::BrakeCmd &cmd)
{
  brake_cmd = cmd;
}

static dbw_mkz_msgs::ThrottleCmd throttle_cmd;

void throttle_callback(const dbw_mkz_msgs::ThrottleCmd &cmd)
{
  throttle_cmd = cmd;
}

RosBridge::RosBridge(std::weak_ptr<uWS::Hub> hub, int argc, char **argv)
  : hub_(hub)
{
  ros::init(argc, argv, "simulator_bridge");
  ros::NodeHandle n;

  dbw_status_pub_ = n.advertise<std_msgs::Bool>(PUBNAME_DBW_STATUS, MSG_QUEUE_SIZE);
  pose_pub_       = n.advertise<geometry_msgs::PoseStamped>(PUBNAME_POSE, MSG_QUEUE_SIZE);
  velocity_pub_   = n.advertise<geometry_msgs::TwistStamped>(PUBNAME_VELOCITY, MSG_QUEUE_SIZE);
  trafficlight_pub_ =
      n.advertise<styx_msgs::TrafficLightArray>(PUBNAME_TRAFFIC_LIGHTS, MSG_QUEUE_SIZE);
  image_pub_ = n.advertise<sensor_msgs::Image>(PUBNAME_IMAGE, 1);
  // NOTE: I can not pass the a callback that points to a non-static member function.
  // Can solve via global variables, not the best thing either...

  final_wpts_sub_ = n.subscribe(SUBNAME_WAYPOINTS, MSG_QUEUE_SIZE, wpts_callback);
  steering_sub_   = n.subscribe(SUBNAME_STEERING, MSG_QUEUE_SIZE, steering_callback);
  brake_sub_      = n.subscribe(SUBNAME_BRAKE, MSG_QUEUE_SIZE, brake_callback);
  throttle_sub_   = n.subscribe(SUBNAME_THROTTLE, MSG_QUEUE_SIZE, throttle_callback);

  ros::spinOnce();
}

void RosBridge::handle_telemetry(const nlohmann::json &data)
{
  publish_changed_dbw_status(data);

  pose_pub_.publish(parse_position(data));

  velocity_pub_.publish(parse_velocities(data));

  // Note: throttle, break, steer commands are being
  // created and emitted in the Server part, not here.

  ros::spinOnce();

  final_wpts_callback(lane_buf);
}

void RosBridge::publish_changed_dbw_status(const nlohmann::json &data)
{
  const bool enabled = data["dbw_enable"];
  if (is_dbw_enabled_ != enabled)
  {
    std_msgs::Bool msg;
    msg.data = enabled;
    dbw_status_pub_.publish(msg);
  }
  is_dbw_enabled_ = enabled;
}

geometry_msgs::PoseStamped RosBridge::parse_position(const nlohmann::json &data) const
{
  const double x          = data["x"];
  const double y          = data["y"];
  const double z          = data["z"];
  const double yaw_degree = data["yaw"];

  return create_pose(x, y, z, yaw_degree);
}

geometry_msgs::TwistStamped RosBridge::parse_velocities(const nlohmann::json &data)
{
  static constexpr auto       ONE_MPH_IN_MS = 0.44704;
  geometry_msgs::TwistStamped velocity;

  // Velocity is measured in the vehicle's CS, so the linear velocity is
  // always along 'x', and the angular one is around 'z' axes.
  // The 'y' is headed to the left, 'z' - up.
  velocity.twist.linear.x  = double(data["velocity"]) * ONE_MPH_IN_MS;
  velocity.twist.angular.z = compute_angular_velocity(double(data["yaw"]));
  return velocity;
}

void RosBridge::handle_traffic_lights(const nlohmann::json &data) const
{
  vector<double> x_points = data["light_pos_x"];
  vector<double> y_points = data["light_pos_y"];
  vector<double> z_points = data["light_pos_z"];
  vector<double> dx = data["light_pos_dx"];
  vector<double> dy = data["light_pos_dy"];
  vector<uint8_t> states = data["light_state"];
  // 0 - red, 1 - yellow, 2 - green

  const bool are_sizes_ok = x_points.size() == y_points.size() &&
                            x_points.size() == z_points.size() &&
                            x_points.size() == states.size() && x_points.size() == dx.size() &&
                            x_points.size() == dy.size();
  if (not are_sizes_ok)
  {
    ROS_ERROR("Traffic light data sizes are not equal!");
    return;
  }

  styx_msgs::TrafficLightArray msg;
  std_msgs::Header        hdr;
  hdr.stamp    = ros::Time::now();
  hdr.frame_id = "/world";
  msg.header = hdr;

  for (size_t i{0}; i < x_points.size(); ++i) {
      const auto x = x_points[i];
      const auto y = y_points[i];
      const auto z = z_points[i];
      const auto yaw = atan2(dy[i], dx[i]);
      const auto state = states[i];
      styx_msgs::TrafficLight light = create_light(x, y, z, yaw, state);
      msg.lights.push_back(light);
  }

  trafficlight_pub_.publish(msg);
}

void RosBridge::handle_camera_image(const nlohmann::json &data) const
{
  if (++img_msg_cnt_ < img_msg_divider_)
  {
    return;
  }
  img_msg_cnt_ = 0;

  const std::string  image_data_str = data["image"];
  const std::string  decoded        = base64_decode(image_data_str);
  vector<uint8_t>    img_data(decoded.begin(), decoded.end());
  cv_bridge::CvImage cv_image_wrapper;
  cv_image_wrapper.encoding = "rgb8";
  cv_image_wrapper.image    = cv::imdecode(cv::Mat(img_data), 1);
  auto msg                  = cv_image_wrapper.toImageMsg();
  msg->header.stamp         = ros::Time::now();
  // NOTE: The image colorspace published is BGR!
  image_pub_.publish(msg);
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

float RosBridge::steering_angle() const
{
  return steering_cmd.steering_wheel_angle_cmd;
}

float RosBridge::throttle_val() const
{
  return throttle_cmd.pedal_cmd;
}

float RosBridge::brake_val() const
{
  return brake_cmd.pedal_cmd;
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

styx_msgs::TrafficLight RosBridge::create_light(double x, double y, double z, double yaw_degree,
                                                uint8_t state) const
{
  styx_msgs::TrafficLight light;
  std_msgs::Header        hdr;
  hdr.stamp    = ros::Time::now();
  hdr.frame_id = "/world";
  light.header = hdr;

  light.pose  = create_pose(x, y, z, yaw_degree);
  light.state = state;

  return light;
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

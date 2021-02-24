#include "rosbridge.h"

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
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>(PUBNAME_POSE, MSG_QUEUE_SIZE);
    velocity_pub_ =
            n.advertise<geometry_msgs::TwistStamped>(PUBNAME_VELOCITY, MSG_QUEUE_SIZE);
}

void RosBridge::handle_telemetry(const nlohmann::json &data)
{
    auto dbw_msg = RosBridge::to_dbw_status(data);
    dbw_status_pub_.publish(dbw_msg);

    auto pose_msg = RosBridge::to_pose(data);
    pose_pub_.publish(pose_msg);

    auto velocity_msg = RosBridge::to_velocity(data);
    velocity_pub_.publish(velocity_msg);
}

std_msgs::Bool RosBridge::to_dbw_status(const nlohmann::json &data)
{
  const bool     enabled = data["dbw_enable"];
  std_msgs::Bool msg;
  msg.data = enabled;
  return msg;
}

geometry_msgs::PoseStamped RosBridge::to_pose(const nlohmann::json &data)
{
  geometry_msgs::PoseStamped msg;
  std_msgs::Header           hdr;
  // TODO: stamp the header with ROS timestamp and other info
  msg.header          = hdr;
  msg.pose.position.x = data["x"];
  msg.pose.position.y = data["y"];
  msg.pose.position.z = data["z"];
  // TODO: quaternion orientation
  return msg;
}

geometry_msgs::TwistStamped RosBridge::to_velocity(const nlohmann::json &data)
{
  geometry_msgs::TwistStamped velocity;
  std_msgs::Header            hdr;
  // TODO: stamp the header with ROS timestamp and other info

  velocity.twist.linear.x = double(data["velocity"]) * 0.44704;  // MPH to m/s

  velocity.twist.angular.z = 0.0;  // TODO: compute angular velocity!
  return velocity;
}

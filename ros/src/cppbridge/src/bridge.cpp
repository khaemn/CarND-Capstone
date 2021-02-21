#include "json.hpp"

#include <dbw_mkz_msgs/SteeringReport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <map>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <string>
#include <uWS/uWS.h>

// for convenience
using nlohmann::json;
using std::string;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1         = s.find_first_of("[");
  auto b2         = s.find_last_of("]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

static constexpr auto MSG_TELEMETRY     = "telemetry";
static constexpr auto MSG_CONTROL       = "control";
static constexpr auto MSG_OBSTACLE      = "obstacle";
static constexpr auto MSG_LIDAR         = "lidar";
static constexpr auto MSG_TRAFFICLIGHTS = "trafficlights";
static constexpr auto MSG_IMAGE         = "image";

// According to styx/conf.py
/*
 'subscribers': [
      {'topic':'/vehicle/steering_cmd', 'type': 'steer_cmd', 'name': 'steering'},
      {'topic':'/vehicle/throttle_cmd', 'type': 'throttle_cmd', 'name': 'throttle'},
      {'topic':'/vehicle/brake_cmd', 'type': 'brake_cmd', 'name': 'brake'},
{'topic':'/final_waypoints', 'type': 'path_draw', 'name': 'path'},
  ],
  'publishers': [
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
  ]
*/
// From telemetry package
static constexpr auto PUBNAME_DBW_STATUS = "/vehicle/dbw_enabled";
static constexpr auto PUBNAME_VELOCITY   = "/current_velocity";
static constexpr auto PUBNAME_POSE       = "/current_pose";

static constexpr auto MSG_QUEUE_SIZE = 20;

static std_msgs::Bool to_dbw_status(const nlohmann::json &data)
{
  const bool enabled = data["dbw_enable"];
  std_msgs::Bool msg;
  msg.data = enabled;
  return msg;
}

static geometry_msgs::PoseStamped to_pose(const nlohmann::json &data) {
  geometry_msgs::PoseStamped msg;
  std_msgs::Header hdr;
  // TODO: stamp the header with ROS timestamp and other info
  msg.header = hdr;
  msg.pose.position.x = data["x"];
  msg.pose.position.y = data["y"];
  msg.pose.position.z = data["z"];
  // TODO: quaternion orientation
  return msg;
}

static geometry_msgs::TwistStamped to_velocity(const nlohmann::json &data)
{
  geometry_msgs::TwistStamped velocity;
  std_msgs::Header            hdr;
  // TODO: stamp the header with ROS timestamp and other info

  velocity.twist.linear.x  = double(data["velocity"]) * 0.44704;  // MPH to m/s

  velocity.twist.angular.z = 0.0;  // TODO: compute angular velocity!
  return velocity;
}

int main(int argc, char **argv)
{
  uWS::Hub h;

  ros::init(argc, argv, "simulator_bridge");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", MSG_QUEUE_SIZE);

  // bridge.publish_dbw_status(dbw_enable)
  // bridge.publish_odometry(data)
  ros::Publisher dbw_status_pub = n.advertise<std_msgs::Bool>(PUBNAME_DBW_STATUS, MSG_QUEUE_SIZE);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>(PUBNAME_POSE, MSG_QUEUE_SIZE);
  ros::Publisher velocity_pub =
      n.advertise<geometry_msgs::TwistStamped>(PUBNAME_VELOCITY, MSG_QUEUE_SIZE);

  bool is_dbw_enabled = true;
  h.onMessage([&dbw_status_pub, &pose_pub, &velocity_pub, &is_dbw_enabled](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    const string inbound      = string(data).substr(0, length);
    const bool   is_msg_valid = (length && length > 2 && data[0] == '4' && data[1] == '2');
    if (!is_msg_valid)
    {
      return;
    }

    auto s = hasData(inbound);
    if (s.empty())
    {
      return;
    }

    auto j = json::parse(s);
    // std::cout << "Message :\n" << inbound << std::endl;

    string event = j[0].get<string>();

    if (event == "telemetry")
    {
      auto dbw_msg = to_dbw_status(j[1]);
      dbw_status_pub.publish(dbw_msg);

      auto pose_msg = to_pose(j[1]);
      pose_pub.publish(pose_msg);

      auto velocity_msg = to_velocity(j[1]);
      velocity_pub.publish(velocity_msg);
//      std_msgs::String pub_msg;

//      pub_msg.data = s;

//      dbw_mkz_msgs::SteeringReport steer_rep;
//      steer_rep.enabled              = true;
//      steer_rep.steering_wheel_angle = 0.666;

//      steer_pub.publish(steer_rep);
    }

    json msgJson;
    msgJson["steering_angle"] = 1.;
    msgJson["throttle"]       = 1.;
    // auto msg = "42[\"steer\"," + msgJson.dump() + "]";
    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    std::cout << "Responce:\n" << msg << std::endl;
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}

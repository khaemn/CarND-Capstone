#include "json.hpp"
#include "rosbridge.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <map>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
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

int main(int argc, char **argv)
{
  auto h = std::make_shared<uWS::Hub>();

  RosBridge rosbridge(std::weak_ptr<uWS::Hub>(h), argc, argv);

  h->onMessage(
      [&rosbridge](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          rosbridge.handle_telemetry(j[1]);
        }

        json msgJson;
        msgJson["steering_angle"] = std::to_string(rosbridge.steering_angle());
        msgJson["throttle"]       = std::to_string(rosbridge.throttle_val());
        msgJson["brake"]          = std::to_string(rosbridge.brake_val());

        auto msg1 = "42[\"steer\"," + msgJson.dump() + "]";
        ws.send(msg1.data(), msg1.length(), uWS::OpCode::TEXT);
        auto msg2 = "42[\"throttle\"," + msgJson.dump() + "]";
        ws.send(msg2.data(), msg2.length(), uWS::OpCode::TEXT);
        auto msg3 = "42[\"brake\"," + msgJson.dump() + "]";
        ws.send(msg3.data(), msg3.length(), uWS::OpCode::TEXT);
        auto msg4 = rosbridge.get_waypoints_tcp_message();
        ws.send(msg4.data(), msg4.length(), uWS::OpCode::TEXT);
      });

  h->onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h->onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h->listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h->run();
}

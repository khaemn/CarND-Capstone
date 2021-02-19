#include "json.hpp"

#include <iostream>
#include <math.h>
#include <ros/ros.h>
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
  uWS::Hub h;

  ros::init(argc, argv, "simulator_bridge");
  ros::NodeHandle n;
  ros::Publisher  chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  h.onMessage([&chatter_pub](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                             uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    const string inbound = string(data).substr(0, length);
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(inbound);
      // std::cout << "Message :\n" << inbound << std::endl;
      std_msgs::String pub_msg;

      pub_msg.data = s;
      // %EndTag(FILL_MESSAGE)%

      // %Tag(ROSCONSOLE)%
      ROS_INFO("%s", pub_msg.data.c_str());
      // %EndTag(ROSCONSOLE)%

      /**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
      // %Tag(PUBLISH)%
      chatter_pub.publish(pub_msg);

      // json msgJson;
      /*msgJson["steering_angle"] = steer_value;
      msgJson["throttle"] = throttle;
      auto msg = "42[\"steer\"," + msgJson.dump() + "]";
      //std::cout << msg << std::endl;
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);return;

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }*/
      json msgJson;
      msgJson["steering_angle"] = 1.;
      msgJson["throttle"]       = 1.;
      // auto msg = "42[\"steer\"," + msgJson.dump() + "]";
      auto msg = "42[\"steer\"," + msgJson.dump() + "]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      std::cout << "Responce:\n" << msg << std::endl;

    }  // end websocket message if
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

#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

constexpr double TARGET_SPEED = 55.0;

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID steering_pid, throttle_pid;
  /**
   * TODO: Initialize the pid variable.
   */

  //        P    I    D
  steering_pid.Init(0.3, 0.0, 15.0);
  throttle_pid.Init(2.0, 0.001, 10.0);

  h.onMessage([&steering_pid, &throttle_pid](uWS::WebSocket<uWS::SERVER> ws,
                    char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          steering_pid.UpdateError(cte, angle);
          steer_value = steering_pid.TotalError();
          steer_value = std::max(-1.0, std::min(1.0, steer_value));

          throttle_pid.UpdateError(speed - TARGET_SPEED);
          double throttle = throttle_pid.TotalError();
          if (throttle > -1.6 && throttle < 0.0 ) {  // don't flash brake lights unneccessarily
            throttle = 0.0;
          }

          // DEBUG
          static int run_count = 0;
          static double steer_min = std::numeric_limits<double>::max();
          static double steer_max = std::numeric_limits<double>::min();
          static double cte_min = std::numeric_limits<double>::max();
          static double cte_max = std::numeric_limits<double>::min();

          ++run_count;
          steer_min = std::min(steer_min, steer_value);
          steer_max = std::max(steer_max, steer_value);
          cte_min = std::min(cte_min, cte);
          cte_max = std::max(cte_max, cte);
          std::cout << run_count << "\tsteer min: " << steer_min << ", max: " << steer_max << ", cte min: " << cte_min << ", max: " << cte_max;
          std::cout << " CTE: " << cte << " Steering Value: " << steer_value << " "; //<< std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          //msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
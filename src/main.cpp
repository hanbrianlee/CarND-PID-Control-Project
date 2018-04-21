#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

//define hyperparameters
#define INNER_LIMIT 0.05 //the inner bandwidth of the hysteresis offset control. Improves the "comfort"
#define OUTER_LIMIT 0.1 //the outer bandwidth of the hysteresis offset control. Improves the "comfort"
#define MAX_STEER_VALUE 0.3
#define MAX_THROTTLE_VALUE 0.3 //this really depends on what the update rate of the sensor information (speed,angle,cte) are, can be higher if update rate is higher. Should be lower if update rate is lower in order to achieve a smoother driving
#define MIN_THROTTLE_VALUE 0.1

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid_s; //for speed
  PID pid_t; //for throttle
  // TODO: Initialize the pid variable
  //and initialize other variables
  pid_s.Init(0.2, 0.004, 3.0);
  pid_t.Init(0.2, 0.004, 3.0);

  //for average speed calculation
  double avg_speed = 0.0;
  double total_speed = 0.0;
  long int it = 0.0;

  h.onMessage([&pid_s, &pid_t, &avg_speed, &total_speed, &it](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value, speed_adj;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          //hysteresis cte adjustment
          if(fabs(cte) < INNER_LIMIT)
          {
            pid_s.limit = OUTER_LIMIT;
          }
          if(fabs(cte) > OUTER_LIMIT)
          {
            pid_s.limit = INNER_LIMIT;
          }

          //if within limit set offset to cte
          if(fabs(cte) < pid_s.limit)
          {
            pid_s.offset = cte;
          }
          //else set offset to 0
          else
          {
            pid_s.offset = 0;
          }

          //update cte with the offset
          cte -= pid_s.offset;

          pid_s.UpdateError(cte);
          //pid_t.UpdateError(cte);
          pid_t.UpdateError(angle);

          //steer_value = pid_s.Kp*cte - pid_s.Kd*((cte-prev_cte)/1) - pid_s.Ki*sum();
          if(pid_s.run_num != 0)
          {
            //run if this is not the first run
            //pid_s.Twiddle();
            //pid_t.Twiddle();
          }

          //diagnose how much noise there is
          if((fabs(pid_s.d_error) > 1) || (fabs(pid_t.d_error) > 15))
          {
             std::cout<< "found abnormality. " << "pid_s.d_error: " << pid_s.d_error << ", pid_t.d_error: " << pid_t.d_error << std::endl;
          }

          //skip PID control if angle measurement or cte measurement has serious jump (noise)
          if((fabs(pid_s.d_error) > 1) || (fabs(pid_t.d_error) > 15))
          {
             steer_value = pid_s.last_steer_value;
             throttle_value = pid_t.last_throttle_value;
          }
          else
          {
            //compute steer_value based on PID control
            steer_value = -pid_s.p[0]*pid_s.p_error - pid_s.p[1]*pid_s.d_error - pid_s.p[2]*pid_s.i_error;
            //apply max steer_value to prevent excessive aggressive steering
            if(fabs(steer_value) > MAX_STEER_VALUE)
            {
              if(steer_value < 0) //prevent braking, deceleration is very uncomfortable for the passenger/driver! =D
              {
                steer_value = -MAX_STEER_VALUE;
              }
              else
              {
                steer_value = MAX_STEER_VALUE;
              }
            }


            //speed adjustment will be made so that the adjustment will be inversely proportional to the error (cte) or (angle?)
            //to ensure that the speed doesn't go too far down. desired speed value of 15
            speed_adj = 0.05*(20-speed); // if way below 20, like 5, then a +0.225 adjustment will be made to throttle.
            //for throttle, 0.3 will be the "0" error line so the car moves at least 0.3 speed
            throttle_value = speed_adj + 0.3 - pid_t.p[0]*pid_t.p_error - pid_t.p[1]*pid_t.d_error - pid_t.p[2]*pid_t.i_error;

            //apply max throttle_value to prevent excessive aggressive throttling
            if(throttle_value > MAX_THROTTLE_VALUE)
            {
              throttle_value = MAX_THROTTLE_VALUE;
            }
            if((throttle_value < 0) && (throttle_value < MIN_THROTTLE_VALUE))
            {
              throttle_value = MIN_THROTTLE_VALUE;
            }


            pid_s.index++;
            pid_t.index++;
            if(pid_s.index == 3) pid_s.index=0; //reset if index reaches 3
            if(pid_t.index == 3) pid_t.index=0; //reset if index reaches 3
            pid_s.run_num++;



            //store last steer_value and throttle_values to use for noise rejection
            pid_s.last_steer_value = steer_value;
            pid_t.last_throttle_value = throttle_value;
          }

          //average speed calculation
          it++;
          total_speed += speed;
          avg_speed = total_speed / it;

          // DEBUG
          //std::cout << "CTE: " << cte << ", Steering Value: " << steer_value << ", angle: " << angle << ", speed: " << speed << std::endl;
          //std::cout << "Original CTE: " << cte + pid_s.offset << ", Offset: " << pid_s.offset << ", New CTE: " << cte << ", limit: " << pid_s.limit << std::endl;
          //std::cout << " speed p0: " << pid_s.p[0] << ",  p1: " << pid_s.p[1] << ",  p2: " << pid_s.p[2] << std::endl;
          //std::cout << "speed dp0: " << pid_s.dp[0] << ", dp1: " << pid_s.dp[1] << ", dp2: " << pid_s.dp[2] << std::endl;
          //std::cout << " throttle p0: " << pid_t.p[0] << ",  p1: " << pid_t.p[1] << ",  p2: " << pid_t.p[2] << std::endl;
          //std::cout << "throttle dp0: " << pid_t.dp[0] << ", dp1: " << pid_t.dp[1] << ", dp2: " << pid_t.dp[2] << std::endl;
          //std::cout << "--------------------------------------------------------" << std::endl;
          std::cout << "avg_speed: " << avg_speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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

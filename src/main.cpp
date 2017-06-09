#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

#include <cmath>

#include "json.hpp"
#include "PID.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// switch for turn on twiddle or not
bool enable_twiddle_angle = false;
bool enable_twiddle_speed = false;

const double cte_limit = 1.8;

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

  PID SteerPID;
  PID SpeedPID;
  
  /*
   * Determination of Kp, Ki, Kd. I got these parameters by twiddling as what is taught in course.
   * First I set Kp, Ki, Kd to zeros and dp[3] = {1,1,1} and enable twiddle. Every time when the cte 
   * is bigger than cte_limit=1.8, it will stop and check the average cte error.
   */
  //double steerP[3] = {2.50853, 0.0119725, 13.4343};
  //double steerDp[3] = {0.001824, 0.000366288, 0.00456821};

  double steerP[3] = {0.918412, 0.0176985, 9.28444};
  double steerDp[3] = {0.00441108, 0.00360905, 0.00813502};
  
  SteerPID.Init(steerP, steerDp);
  SteerPID.ResetState();
  
  /* The initial value for Speed parameters are Kp = 0.5, Ki = 0, Kd = 1 */
  /* It also be twiddled b as what is taught in course */
  //double speedP[3] = {2.40481, 0.0, 12.2864};
  //double speedDp[3] = {0.12718, 0.0381521, 0.428237};
  double speedP[3] = {3.53998, 0.0, -1.05401};
  double speedDp[3] = {0.505395, 0.185302, 0.375914};
  
  SpeedPID.Init(speedP, speedDp);
  SpeedPID.ResetState();

  if (enable_twiddle_angle) {
      SteerPID.twiddle();
  }
  if (enable_twiddle_speed) {
      SpeedPID.twiddle();
  }
  
  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_angle, steer_speed;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          bool bSteerPIDCnt = SteerPID.UpdateCnt();
          bool bSpeedPIDCnt = SpeedPID.UpdateCnt();

          if ((enable_twiddle_angle || enable_twiddle_speed ) && (cte > cte_limit || !bSteerPIDCnt || !bSpeedPIDCnt)) {
            //cout << "bSpeedPIDCnt=" << bSpeedPIDCnt << " steps_cnt=" << SpeedPID.steps_cnt << endl;
            //cout << "cte=" << cte << " cte_limit=" << cte_limit << endl;
            
            if (enable_twiddle_angle) {
                SteerPID.TotalError();
                SteerPID.notifying();
            }
            
            if (enable_twiddle_speed) {
                SpeedPID.TotalError();
                SpeedPID.notifying();
            }
            

            while (SteerPID.error_updated || SpeedPID.error_updated)
              std::this_thread::sleep_for(std::chrono::milliseconds(1));

            /* send reset command */
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

            SteerPID.ResetState();
            SpeedPID.ResetState();

            //cout<<reset_msg<<endl;
          } else {

            SteerPID.UpdateError(cte);
            steer_angle = SteerPID.Steering();

            /* Get hint from 
             * https://github.com/jendrikjoe/UdacityProjects/blob/master/src/main.cpp
             */
            double target_speed = 30.*(1.-abs(steer_angle)) + 20.;
            SpeedPID.UpdateError(speed - target_speed);
            steer_speed = SpeedPID.Steering();

            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << steer_angle << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_angle;
            msgJson["throttle"] = steer_speed;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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
  
  /* wait for adjust pid to complete */
  SteerPID.adjust_pid.join();
  SpeedPID.adjust_pid.join();
  
}

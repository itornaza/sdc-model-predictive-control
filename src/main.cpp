#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "MPC.h"
#include "helpers.h"

using namespace std;

// For convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;
  MPC mpc;
  
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          
          // TODO: Use the values from constants.h after resolving symbol issues
          const double latency = 0.1; // In seconds
          const double Lf = 2.67;
          
          //-------------------------
          // Get data from simulator
          //-------------------------
          
          // Detail listing of simulator responses at:
          // https://github.com/udacity/CarND-MPC-Project/blob/master/DATA.md
          
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];
          double cte;
          double epsi;
          
          //-------------------------
          // Calculate {δ, α}
          //-------------------------
          
          // Transform points (ptsx, ptsy) from map to car coordinates
          for (int ix = 0; ix < ptsx.size(); ++ix) {
            // Shift car reference angle to 90 degrees clockwise
            double shift_x = ptsx[ix] - px;
            double shift_y = ptsy[ix] - py;
            ptsx[ix] = shift_x * cos(-psi) - shift_y * sin(-psi);
            ptsy[ix] = shift_x * sin(-psi) + shift_y * cos(-psi);
          }
          
          // Transform ptsx and ptsy arrays into VectorXd
          Eigen::Map<Eigen::VectorXd> ptsx_transform(&ptsx[0], 6);
          Eigen::Map<Eigen::VectorXd> ptsy_transform(&ptsy[0], 6);
          
          // Create the polynomial to fit the given points
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
          
          // State variables without latency
          px = 0.0;   // New origin at (0,0)
          py = 0.0;
          psi = 0.0;
          v *= 0.44704; // Convert to m/sec for the calculations
          cte = coeffs[0]; // TODO: same as polyeval(coeffs, 0.0);
          epsi = -atan(coeffs[1]);
          
          // Adjust the state variables to account for latency
          
          // If δ is positive we rotate counter-clockwise, or turn left.
          // In the simulator however, a positive value implies a right turn and
          // a negative value implies a left turn. Thus, we multiply the
          // steering value by -1 for compatibility with the equations
          
          px = v * latency; // cos(0.0) = 1.0
          py = 0.0; // sin(0.0) = 0.0
          psi = v * (-steer_value) * latency / Lf;
          v += throttle_value * latency;
          cte += v * sin(epsi) * latency;
          epsi += v * (-steer_value) * latency / Lf;
          
          // Define the state
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          
          // Calculate steering angle and throttle using MPC. Both are in
          // between [-1, 1] and are contained in the returned vars array
          auto vars = mpc.Solve(state, coeffs);
          
          //-------------------------
          // Lines to display
          //-------------------------
         
          // Set up the waypoints/reference yellow line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_inc = 2.5;
          double num_points = 25;
          for (int ix = 1; ix < num_points; ++ix) {
            next_x_vals.push_back(poly_inc * ix);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * ix));
          }
          
          // Set up the MPC predicted trajectory green line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (int ix = 2; ix < vars.size(); ++ix) {
            if (ix % 2 == 0) {
              mpc_x_vals.push_back(vars[ix]);
            } else {
              mpc_y_vals.push_back(vars[ix]);
            }
          }
          
          //-------------------------
          // Send data to simulator
          //-------------------------
          
          // Prepare the json message
          json msgJson;
          
          // Steering angle
          // Divide by deg2rad(25) before sending the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25)]
          // instead of [-1, 1]
          msgJson["steering_angle"] = vars[0] / (deg2rad(25));
          msgJson["throttle"] = vars[1];

          // Add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system the points in the simulator are
          // connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          // Add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system the points in the simulator are
          // connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          cout << msg << endl;
          
          // Latency simulation by sleeping!
          this_thread::sleep_for(chrono::milliseconds(
                                 static_cast<int>(latency * 1000)));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // End - Telemetry
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      } // End if/else - Auto or manual driving
    } // End if - Data checks
  });

  /**
   * We don't need this since we're not using HTTP but if it's removed the
   * program doesn't compile :-(
   */
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // I guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    cout << "Listening to port " << port << endl;
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}

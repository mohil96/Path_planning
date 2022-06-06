#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
//TODO:add headers (lines 11-12)
#include "fsm.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

//TODO:add constants(lines 20-32)
const double MAX_VEL = 49.5;       // MPH
const double MAX_ACC = .224;
const double MAX_DEC = .448;
const int LEFT_LANE = 0;
const int MIDDLE_LANE = 1;
const int RIGHT_LANE = 2;
const int INVALID_LANE = -1;

const int LEFT_LANE_MAX = 4;
const int MIDDLE_LANE_MAX = 8;
const int RIGHT_LANE_MAX = 12;

const int PROJECTION_IN_METERS = 30;

//TODO: add states(lines 35-45)
// Defining Fixed States for FSM
enum States { Stay_Lane, Lane_Change_Left, Lane_Change_Right, Wait_Lane_Change_Left, Wait_Lane_Change_Right };
enum Triggers {  CarAhead, Clear };
FSM::Fsm<States, States::Stay_Lane, Triggers> fsm;
const char * StateNames[] = { "Stay in Lane", "Prepare for Change to Left Lane", "Prepare for Change to Right Lane", "Change To Left Lane", "Change To Right Lane" };

void dbg_fsm(States from_state, States to_state, Triggers trigger) {
    if (from_state != to_state) {
        std::cout << "State Changed To: " << StateNames[to_state] << "\n";
    }
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

// start in lane 1
  int lane = 1;   // middle Lane 
  
  // reference velocity
  double ref_vel = 0.0; //mph

  bool car_ahead = false;
  bool car_left = false;
  bool car_right = false;
  bool car_ahead_speed = false;
  bool car_left_speed = false;
  bool car_right_speed = false;
  


  //TODO: State Machine Setup(lines95-112)
  fsm.add_transitions({
                            //  from state ,to state  ,triggers        ,guard                    ,action
                            { States::Stay_Lane  ,States::Wait_Lane_Change_Left ,Triggers::CarAhead  ,[&]{return car_ahead && car_left && car_ahead_speed && car_left_speed && lane > LEFT_LANE;}  ,[&]{ref_vel -= MAX_DEC;} },
                            { States::Wait_Lane_Change_Left  ,States::Wait_Lane_Change_Left ,Triggers::CarAhead  ,[&]{return true ;}  ,[&]{ref_vel -= MAX_DEC;} },
                            { States::Wait_Lane_Change_Left  ,States:: Lane_Change_Left ,Triggers::CarAhead  ,[&]{return car_ahead && !car_left && lane > LEFT_LANE;}  ,[&]{lane--;} },
                            { States::Wait_Lane_Change_Left  ,States:: Stay_Lane ,Triggers::Clear  ,[&]{return true;}  ,[&]{if (ref_vel < MAX_VEL) {ref_vel += MAX_ACC;}} },
                            { States::Lane_Change_Left  ,States:: Stay_Lane ,Triggers::CarAhead  ,[&]{return true;}  ,[&]{ref_vel -= MAX_DEC;} },
                            { States::Wait_Lane_Change_Left ,States::Lane_Change_Left ,Triggers::CarAhead  ,[&]{return car_ahead && car_left && !car_left_speed && lane > LEFT_LANE ;}  ,[&]{ lane--; } },
                            { States::Lane_Change_Left ,States::Stay_Lane ,Triggers::Clear  ,[&]{return !car_ahead;}  ,[&]{if (ref_vel < MAX_VEL) {ref_vel += MAX_ACC;}} },
                            { States::Stay_Lane ,States::Lane_Change_Left ,Triggers::CarAhead  ,[&]{return car_ahead && !car_left && lane > LEFT_LANE;}  ,[&]{lane--;} },

                            { States::Stay_Lane  ,States::Wait_Lane_Change_Right ,Triggers::CarAhead  ,[&]{return car_ahead && car_right && car_ahead_speed && car_right_speed && lane < RIGHT_LANE ;}  ,[&]{ref_vel -= MAX_DEC;} },
                            { States::Wait_Lane_Change_Right  ,States::Wait_Lane_Change_Right ,Triggers::CarAhead  ,[&]{return true ;}  ,[&]{ref_vel -= MAX_DEC;} },
                            { States::Wait_Lane_Change_Right  ,States:: Stay_Lane ,Triggers::Clear  ,[&]{return true;}  ,[&]{if (ref_vel < MAX_VEL) {ref_vel += MAX_ACC;}} },
                            { States::Wait_Lane_Change_Right  ,States:: Lane_Change_Right ,Triggers::CarAhead  ,[&]{return car_ahead && !car_right && lane < RIGHT_LANE;}  ,[&]{lane++;} },
                            { States::Wait_Lane_Change_Right ,States::Lane_Change_Right ,Triggers::CarAhead  ,[&]{return car_ahead && car_right && !car_right_speed && lane < RIGHT_LANE ;}  ,[&]{ lane++; } },
                            { States::Lane_Change_Right ,States::Stay_Lane ,Triggers::Clear  ,[&]{return !car_ahead;}  ,[&]{if (ref_vel < MAX_VEL) {ref_vel += MAX_ACC;}} },
                            { States::Lane_Change_Right ,States::Stay_Lane ,Triggers::CarAhead  ,[&]{return true;}  ,[&]{ref_vel -=1.5* MAX_DEC;} },
                            { States::Stay_Lane ,States::Lane_Change_Right ,Triggers::CarAhead  ,[&]{return car_ahead && !car_right && lane < RIGHT_LANE;}  ,[&]{lane++;} },
                            
                            { States::Stay_Lane ,States::Stay_Lane ,Triggers::Clear  ,[&]{return !car_ahead;}  ,[&]{if (ref_vel < MAX_VEL) {ref_vel += MAX_ACC;}} }
                            
                    });

  fsm.add_debug_fn(dbg_fsm);

//TODO:add function arguments(car ahead,carleft,car right,lane) (lines 117-118)
  h.onMessage([&car_ahead, &car_ahead_speed, &car_left, &car_left_speed, &car_right, &car_right_speed, &ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds (lines 164-337)
           */
            int prev_size = previous_path_x.size();

            if (prev_size > 0) {
                car_s = end_path_s;
            }

            // 1. PREDICTION : Analysing other cars positions.
            car_ahead = false;
            car_ahead_speed = false;
            car_left = false;
            car_left_speed = false;
            car_right = false;
            car_right_speed = false;
            
            // Sensor Fusion stores other Car's state: [id, x, y, vx, vy, s, d]

            for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                float d = sensor_fusion[i][6];
                int other_car_lane = INVALID_LANE;

                // Determine the lane of the other car
                if ( d > 0 && d < LEFT_LANE_MAX ) {
                    other_car_lane = LEFT_LANE;
                } else if ( d > LEFT_LANE_MAX && d < MIDDLE_LANE_MAX ) {
                    other_car_lane = MIDDLE_LANE;
                } else if ( d > MIDDLE_LANE_MAX && d < RIGHT_LANE_MAX ) {
                    other_car_lane = RIGHT_LANE;
                }
                if (other_car_lane == INVALID_LANE) {
                    continue;
                }

                // Determine the speed of the other car
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                // Estimate the other car's position after executing previous trajectory
                check_car_s += (double) prev_size * 0.02 * check_speed;

                if ( other_car_lane == lane ) {
                    // Other car is in the same lane
                    car_ahead |= check_car_s > car_s && check_car_s - car_s < PROJECTION_IN_METERS;
                    car_ahead_speed |= (ref_vel - check_speed) > 5;
                } else if ( other_car_lane - lane == -1 ) {
                    // Other car is on the left lane
                    car_left |= car_s - PROJECTION_IN_METERS < check_car_s && car_s + PROJECTION_IN_METERS > check_car_s;
                    car_left_speed |= (ref_vel - check_speed) > 5;
                } else if ( other_car_lane - lane == 1 ) {
                    // Other car is on the right lane
                    car_right |= car_s - PROJECTION_IN_METERS < check_car_s && car_s + PROJECTION_IN_METERS > check_car_s;
                    car_right_speed |= (ref_vel - check_speed) > 5;
                }
            }

            // 2. BEHAVIOR: Trigger State Changes Depending If Road Clear of Vehicle Ahead
            if (car_ahead) {
                // Execute 'CarAhead' trigger on state machine
                fsm.execute(Triggers::CarAhead);
            } else {
                // Execute 'Clear' trigger on state machine
                fsm.execute(Triggers::Clear);
            }


            // 3. TRAJECTORY GENERATION: Create a smooth trajectory for the car to follow

            // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // later interpolate waypoints with spline and fill in more waypoints

            vector<double> pts_for_spline_x;
            vector<double> pts_for_spline_y;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // If previous size is almost empty, use car as starting reference
            if (prev_size < 2) {
                // Use two points that make the path tangent to the car 
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                pts_for_spline_x.push_back(prev_car_x);
                pts_for_spline_x.push_back(car_x);

                pts_for_spline_y.push_back(prev_car_y);
                pts_for_spline_y.push_back(car_y);
            }
            // Use previous path's points as starting reference
            else {
                // Redefine reference state as previous path endpoint
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];

                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

                // Use two points that make the path tangent to the previous path's endpoint
                pts_for_spline_x.push_back(ref_x_prev);
                pts_for_spline_x.push_back(ref_x);

                pts_for_spline_y.push_back(ref_y_prev);
                pts_for_spline_y.push_back(ref_y);
            }

            // In Frenet add 3 waypoints evenly 30m spaced points ahead of the starting reference
            vector<double> next_wp0 = getXY(car_s+PROJECTION_IN_METERS, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+(PROJECTION_IN_METERS*2), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+(PROJECTION_IN_METERS*3), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            pts_for_spline_x.push_back(next_wp0[0]);
            pts_for_spline_x.push_back(next_wp1[0]);
            pts_for_spline_x.push_back(next_wp2[0]);

            pts_for_spline_y.push_back(next_wp0[1]);
            pts_for_spline_y.push_back(next_wp1[1]);
            pts_for_spline_y.push_back(next_wp2[1]);

            for (int i=0; i<pts_for_spline_x.size(); i++) {
                // Shift car angle reference to 0 degrees in order to stay at the center of the lane
                double shift_x = pts_for_spline_x[i]-ref_x;
                double shift_y = pts_for_spline_y[i]-ref_y;

                pts_for_spline_x[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                pts_for_spline_y[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

            // Create a spline
            tk::spline s;

            // Set (x,y) points to the spline
            s.set_points(pts_for_spline_x, pts_for_spline_y);

            // Define the actual (x,y) points that will be used for the planner
            vector<double> trajectory_x;
            vector<double> trajectory_y;

            // Start with all of the previous path points from last time
            for (int i=0; i < previous_path_x.size(); i++) {
                trajectory_x.push_back(previous_path_x[i]);
                trajectory_y.push_back(previous_path_y[i]);
            }

            // Calculate how to break up spline points so that the desired refrence velocity is kept
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;

            // Fill up the rest of the path planner after filling it with previous points
            // Always 50 points will be output
            for (int i=0; i <= 50-previous_path_x.size(); i++) {

                if ( ref_vel > MAX_VEL ) {
                    ref_vel = MAX_VEL;
                } else if ( ref_vel < MAX_ACC ) {
                    ref_vel = MAX_ACC;
                }

                double N = (target_dist/(0.02*ref_vel/2.24));
                double x_point = x_add_on+(target_x)/N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                //Rotate back to normal after rotating earlier
                x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;
                
                trajectory_x.push_back(x_point);
                trajectory_y.push_back(y_point);
            }
           


          msgJson["next_x"] = trajectory_x;
          msgJson["next_y"] = trajectory_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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

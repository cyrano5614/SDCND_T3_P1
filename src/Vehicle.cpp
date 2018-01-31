#include "Vehicle.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Vehicle::Vehicle() {}

// Vehicle::~Vehicle() {}

void Vehicle::lane_logic(vector<vector<double> > sensor_fusion, double car_s, int prev_size){

  // if (!too_close) {
  //   vehicle_infront_speed = speed_limit;
  // }
  too_close = false;
  left_close = false;
  right_close = false;
  /* dist = 999999; */
  int front_safe_distance = 40;
  // int switch_safe_distance = 30;
  int rear_safe_distance = 10;
  
  //Check if vehicle is too close infront of the agent
  for (unsigned int i = 0; i < sensor_fusion.size(); i++)
  {

    float d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = sensor_fusion[i][5];

    /* double predict_car_s = (double)prev_size * 0.02 * check_speed; */
    check_car_s += (double)prev_size * 0.02 * check_speed;
    dist = check_car_s - car_s;

    //Check if vehicle is in the same lane
    if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
    {
      //Safe distance set to 30m
      if ((dist > 0) && (dist < front_safe_distance))
      {
        if (dist < front_safe_distance) {
          too_close = true;
          vehicle_infront_speed = check_speed;
        }
        // else if (dist < switch_safe_distance) {
        //   vehicle_infront_speed = check_speed;
        // }
        /* too_close = true; */
        /* vehicle_infront_speed = check_speed; */
      }
      // else {
      //   dist = 999999;
      //   too_close = false;
      // }
    }

    //Check the status of the lane left of the agent
    else if ((lane != 0) && (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2)))
    {
      if ((check_car_s > car_s) && ((dist) < front_safe_distance))
      {
        left_close = true;
      }
      else if ((check_car_s < car_s) && ((-dist) < rear_safe_distance))
      {
        left_close = true;
      }
    }

    //Check the status of the lane left of the agent
    else if ((lane != 2) && (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2)))
    {
      if ((check_car_s > car_s) && ((dist) < front_safe_distance))
      {
        right_close = true;
      }
      else if ((check_car_s < car_s) && ((-dist) < rear_safe_distance))
      {
        right_close = true;
      }
    }
    
    //Lane logic
    if (lane == 0 && too_close && !right_close) {
      lane = 1;
    }
    else if (lane == 2 && too_close && !left_close) {
      lane = 1;
    }
    else if (lane == 1 && too_close && !left_close) {
      lane = 0;
    }
    else if (lane == 1 && too_close && !right_close) {
      lane = 2;
    }


      




  }

}



#include "Vehicle.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

/* Vehicle::Vehicle() {} */

/* Vehicle::~Vehicle() {} */

void Vehicle::lane_logic(vector<vector<double> > sensor_fusion, double car_s, int prev_size){

  //Check if vehicle is too close infront of the agent
  for (uint i = 0; i < sensor_fusion.size(); i++)
  {

    float d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = sensor_fusion[i][5];

    double predict_car_s = prev_size * 0.02 * check_speed;

    //Check if vehicle is in the same lane
    if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
    {
      //Safe distance set to 30m
      if ((predict_car_s > car_s) && ((predict_car_s - car_s) < 30))
      {
        too_close = true;
        vehicle_infront_speed = check_speed;
      }
      else {
        too_close = false;
      }
    }

    //Check the status of the lane left of the agent
    else if ((lane != 0) && (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2)))
    {
      if ((predict_car_s > car_s) && ((predict_car_s - car_s) < 30))
      {
        left_close = true;
      }
      else if ((predict_car_s < car_s) && ((car_s - predict_car_s) < 20))
      {
        left_close = true;
      }
      else {
        left_close = false;
      }
    }

    //Check the status of the lane left of the agent
    else if ((lane != 2) && (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2)))
    {
      if ((predict_car_s > car_s) && ((predict_car_s - car_s) < 30))
      {
        right_close = true;
      }
      else if ((predict_car_s < car_s) && ((car_s - predict_car_s) < 20))
      {
        right_close = true;
      }
      else {
        right_close = false;
      }
    }
  }

}



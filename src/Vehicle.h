#ifndef Vehicle_H
#define Vehicle_H

#include <iostream>
#include <vector>

using namespace std;

class Vehicle
{
  public:

    int lane = 1;
    double speed_limit = 49.5;

    bool too_close = false;
    bool left_close = false;
    bool right_close = false;
    double vehicle_infront_speed = 49;
    double dist;

    Vehicle(){};

    ~Vehicle(){};

    void lane_logic(vector<vector<double>> sensor_fusion, double car_s, int prev_size);

};

#endif /* Vehicle_H */

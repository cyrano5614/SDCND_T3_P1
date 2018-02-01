# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Project Introduction ###

-------------------------------------------------------------------------------

The goal of this project is to safely navigate around a virtual highway with other virtual cars that is also driving +-10 MPH of the 50 MPH speed limit.  The vehicle must meet below criterias.

1. The car is able to drive 4.32 miles without incident, which is roughly one loop of the virtual highway.
2. The car drives according to the speed limit.
3. Max Acceleration and Jerk are not exceeded.
4. Car does not have collisions.
5. The car stays in the lane, except for the time between changing lanes.
6. The car is able to change lanes.

-------------------------------------------------------------------------------

### Project Implementation ###

-------------------------------------------------------------------------------

#### Data ####
Data of the map is read from the csv file provided.  The waypoint list contains [x,y,s,dx,dy] values.  X and y are the waypoint's map coordinate position and the s value is the distance following the road.

|        x |         y |                s |           dx |         dy |
|  :-----: | :-------: |       :--------: |     :-------: |  :-------: |
| 784.6001 |  1135.571 |                0 |  -0.02359831 | -0.9997216 |
| 815.2679 |   1134.93 | 30.6744785308838 |  -0.01099479 | -0.9999396 |
| 844.6398 |  1134.911 | 60.0463714599609 | -0.002048373 | -0.9999979 |
| 875.0436 |  1134.808 | 90.4504146575928 | -0.001847863 | -0.9999983 |
|  905.283 |  1134.799 | 120.689735412598 |  0.004131136 | -0.9999915 |
 
As seen above, the waypoints are separated approximately 30 meters from each other looking at the s values.

The main vehicle's localization data is given with no noise.

* x: The car's x position in map coordinates

* y: The car's y position in map coordinates

* s: The car's s position in frenet coordinates

* d: The car's d position in frenet coordinates

* yaw: The car's yaw angle in the map

* speed: The car's speed in MPH

Second crucial information comes from the sensor fusion data of the other vehicles that are in the same lane.  The sensor fusion data has no noise, and returns following 6 variables.

* id: Unique Identifier of a vehicle

* x : Global x position

* y : Global y position

* vx: x velocity component

* vy: y velocity component

* s : Frenet Coordinate

* d : Frenet Coordinate

-------------------------------------------------------------------------------

#### Vehicle Logic ####
The sensor fusion information was used as an input along with the main vehicle's s position to update the current situation of the main vehicle.  Each vehicle in the sensor fusion data was fed into series of logics to determine if the vehicle was too close in the front and to left or right. 

``

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



#### Trajectory ####

-------------------------------------------------------------------------------

Now that we have determined the what action the main vehicle would take, it is time to lay out the trajectory of the vehicle.  First, we generate few points ahead of vehicle by incorporating the waypoints provided by the map data.
``

    vector<double> next_wp0 = getXY(car_s + 40, (2+4*agent.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 80, (2+4*agent.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 120, (2+4*agent.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

From above code, we generated 3 points, each 40 meters apart to sparse map out our new trajectory.  Next we use a handy spline.h file to fill the trajectory points with a smooth spline.  It is also worth noting that the equation takes the lane position of main vehicle, agent.lane, into consideration.  This was calculated at the previous step in Vehicle Logic.
``

            for (int i = 1; i <= 50-previous_path_x.size(); i++)
            {
              double N = (target_dist/(0.02 * ref_speed / 2.24));
              double x_point = x_add_on + (target_x) / N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

As seen from the code above, we filled the trajectory with 50 evenly spaced waypoints.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)

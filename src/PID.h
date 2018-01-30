#ifndef PID_H
#define PID_H

#include <chrono>
#include <cmath>

class PID
{
  public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    /* auto prev_time; */
    /* auto current_time; */
    std::chrono::time_point<std::chrono::system_clock> prev_time;
    std::chrono::time_point<std::chrono::system_clock> current_time;

    bool has_old_d_error;

    double old_d_error;
    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
};

#endif /* PID_H */

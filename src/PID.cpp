#include "PID.h"

#include <chrono>
#include <cmath>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  prev_time = std::chrono::system_clock::now();

  has_old_d_error = false;
}

void PID::UpdateError(double cte)
{
  if (!has_old_d_error)
  {
    old_d_error = cte;

    has_old_d_error = true;
  }
  current_time = std::chrono::system_clock::now();

  std::chrono::duration<double> interval = current_time - prev_time;
  
  p_error = cte;
  d_error = (cte - old_d_error) / interval.count();
  i_error += cte;

  prev_time = current_time;
  old_d_error = cte;

}

double PID::TotalError()
{
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}

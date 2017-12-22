#include "PID.h"

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
}

void PID::UpdateError(double cte)
{
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

void PID::TotalError()
{
  if (speed == 0)
  {
    speed = 0.001;
  }

  double p_speed = 0.000;
  double i_speed = 0.00000;
  double d_speed = 0.00;
  if (steer)
  {
    return -((Kp - p_speed * speed) * p_error) - ((Ki - i_speed * speed) * i_error)
           - ((Kd + d_speed * speed)* d_error);
  }
  else
  {
    return -((Kp + p_speed * speed) * p_error) - (Ki * i_error) - ((Kd - d_speed * speed) * d_error);
  }
}

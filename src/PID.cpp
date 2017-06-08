#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
    :p_error(0)
    ,d_error(0)
    ,i_error(0)
    ,Kp(0)
    ,Ki(0)
    ,Kd(0)
 {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0;
    d_error = 0;
    i_error = 0;
}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;
    p_error = cte;
    
    i_error += cte;
}

double PID::Steering() {
  double steering = -Kp * p_error - Ki * i_error - Kd * d_error;
  steering = steering < -1.0 ? -1.0 : steering;
  steering = steering > 1.0 ? 1.0 : steering;
  
  return steering;
}

double PID::TotalError() {

}

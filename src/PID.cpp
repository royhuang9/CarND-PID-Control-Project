#include <iostream>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <thread>

#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
    :p_error(0)
    ,d_error(0)
    ,i_error(0)
    ,Kp(p[0])
    ,Ki(p[1])
    ,Kd(p[2])
 {}

PID::~PID() {}

void PID::Init(double p[], double dp[]) {
    this->Kp = p[0];
    this->Ki = p[1];
    this->Kd = p[2];
    
    this->dp[0] = dp[0];
    this->dp[1] = dp[1];
    this->dp[2] = dp[2];
}

void PID::ResetState()
{     
    p_error = 0;
    d_error = 0;
    i_error = 0;
    
    total_error = 0;
    error = HUGE_VAL;
    
    steps_th = 100;
    steps_lmt = 800;
    steps_cnt = 0;
    
    first_step = true;
    error_updated = false;
}

void PID::UpdateError(double cte) {
    if (first_step) {
        p_error = cte;
        first_step = false;
    }

    d_error = cte - p_error;
    p_error = cte;

    i_error += cte;
    
    if (steps_cnt > steps_th) {
        total_error += cte*cte;
    }
}

double PID::Steering() {
  double steering = -Kp * p_error - Ki * i_error - Kd * d_error;
  steering = steering < -1.0 ? -1.0 : steering;
  steering = steering > 1.0 ? 1.0 : steering;
  
  return steering;
}

double PID::TotalError() {
    if (steps_cnt < steps_lmt) {
        total_error = HUGE_VAL;
    }
    error = total_error / (steps_cnt - steps_th);
    cout << "step_cnt:" << steps_cnt << " total_error:"<<total_error<<" error:"<<error<<endl;
    return error;
}

void PID::twiddle()
{
    /* Adjust pid according to error */
    adjust_pid=thread([&]() {
      /* notify onMessage to run */
      cout<<"Thread adjust_pid started" << endl;
      error_updated = false;
      wait_for_error();

      best_err = error;

      unsigned int it = 0;
      while ((dp[0] + dp[1] + dp[2]) > 0.002) {
        cout << "\nIteration " << it << " best_err: "<<best_err<<endl;

        for(int i=0; i < 3; i++) {
          p[i] += dp[i];
          ResetState();
          cout<<"\nKp=" << Kp << " Ki=" << Ki << " Kd=" << Kd << endl;
          cout<<"dp[0]=" << dp[0] << " dp[1]=" << dp[1] << " dp[2]=" << dp[2] << endl;

          /* notify onMessage to run */
          error_updated = false;
          wait_for_error();

          if (error < best_err) {
            best_err = error;
            dp[i] *= 1.1;
            
          } else {
            p[i] -= 2* dp[i];

            cout<<"\nKp=" << Kp << " Ki=" << Ki << " Kd=" << Kd << endl;
            cout<<"dp[0]=" << dp[0] << " dp[1]=" << dp[1] << " dp[2]=" << dp[2] << endl;


            error_updated  = false;
            wait_for_error();

            if (error < best_err){
              best_err = error;
              dp[i] *= 1.1;

            } else {
              p[i] += dp[i];
              dp[i] *= 0.9;
            }
          }
        }
        it ++;
      }

        cout<<"Thread adjust_pid exit"<<endl;
    });

}

void PID::notifying() {
    std::unique_lock<std::mutex> lock(m);
    error_updated = true;
    cond_var.notify_one();
}

void PID::wait_for_error() {
    std::unique_lock<std::mutex> lock(m);
        while (!error_updated) {
            cond_var.wait(lock);
    }
}

bool PID::UpdateCnt()
{
    if (++steps_cnt > steps_lmt)
        return false;
    return true;
}
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  state = 0;
  index = 0;
  best_error = 1000.0;

  p = {Kp, Kd, Ki};
  dp = {0.01*Kp ,0.01*Kd, 0.01*Ki};
  run_num=0;

  offset = 0.0;
  limit = 0.0;

  last_steer_value = 0.0;
  last_throttle_value = 0.0;
}

void PID::UpdateError(double err) {
  // d_error is difference from old cte (p_error) to the new cte
  d_error = (err - p_error);
  // p_error gets set to the new cte
  p_error = err;
  // i_error is the sum of ctes to this point
  i_error += err;


}

double PID::TotalError() {
  //p_error would be the same as "new cte received" assuming that this function gets called after the UpdateError function in the main
  return dp[0] + dp[1] + dp[2];
}

void PID::Twiddle() {
  //apply twiddle algorithm to fine-tune best parameters for Kp, Kd, Ki
  //this algorithm runs to tweak Kp, Kd, Ki every time-step which is a bit different from the twiddle algorithm
  //taught in the udacity course, which runs the twiddle algorithm over the whole run (n steps) over multiple iterations of the runs.
  double tol = 0.001;

  if(TotalError() > tol)
  {
    //need to implement "state memory" since we are running tied to the car simulator.
    //otherwise the twiddle algorithm would have to be run multiple iterations with the simulator running many many times

    //if the first/default state
    if(state == 0)
    {
      p[index] += dp[index];
      if(p_error < best_error)
      {
        best_error = p_error;
        dp[index] *= 1.1;
      }
      else
      {
        p[index] -= 2*dp[index];
        state = 1;
      }
    }

    //if the second/last state
    else //if state is 1
    {
      if(p_error < best_error)
      {
        best_error = p_error;
        dp[index] *= 1.1;
      }
      else
      {
        p[index] += dp[index];
        dp[index] *= 0.9;
      }
      state = 0; //reset state to 0
    }
  }
}


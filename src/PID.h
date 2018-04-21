#ifndef PID_H
#define PID_H
#include<vector>

class PID {
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

  //the following are needed for the "state" implementation for Twiddle
  int state; //default state right after the first p[i]+=dp[i], and the last state which should be at right after p[i] -= 2*dp[i]
  int index;
  double best_error;
  std::vector<double> p;
  std::vector<double> dp;
  long int run_num;

  //used for hyesteresis cte offset control
  double offset;
  double limit;

  //used for severe noise rejection
  double last_steer_value;
  double last_throttle_value;

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
  void UpdateError(double err);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle();
};

#endif /* PID_H */

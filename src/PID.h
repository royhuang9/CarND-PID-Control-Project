#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double dp[3];
  double p[3];
  /*
  * Coefficients
  */ 
  /* Kp is the proportional to the CTE */
  double &Kp;
  
  /* Ki is the integration parameter, it accumulate all the errors */
  double &Ki;
  
  /* Kd is the derivative paramter, it refl */
  double &Kd;

  double total_error;
  double error;
  double best_err;

  unsigned int steps_th;
  unsigned int steps_lmt;
  unsigned int steps_cnt;
  
  bool first_step;  
  bool error_updated;

 /* mutex and conditional variable */
  std::mutex m;
  
  std::condition_variable cond_var;
  
  std::thread adjust_pid;

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
  void Init(double p[], double dp[]);
  
  void ResetState();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  double Steering();
  
  void twiddle();
  
  void notifying();
  void wait_for_error();
  
  /* return false when steps_count larger than steps_lmt */
  bool UpdateCnt();
};

#endif /* PID_H */

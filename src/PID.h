#ifndef PID_H
#define PID_H
#include <vector>


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

  /*
  * Twiddle variables
  */
  std::vector<double> dp;
  std::vector<double> p;

  int pid_index;
  
  // number of iterations 
  int n_iter;
  int iter;

  double diff_cte ;
  double incr_cte ; 
  double total_error;
  double best_error;


  bool twiddle_adding;
  bool twiddle_subtracting;

  double prev_time;

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
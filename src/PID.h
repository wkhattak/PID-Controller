#ifndef PID_H
#define PID_H
#include <array>
#include <string>

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;
  double cte_;
  double prev_cte_;
  double integ_cte_;
  double diff_cte_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;
  
  /*
  * Init flag
  */ 
  bool init_;
  
  /*
  * Identifier
  */ 
  std::string id_;
  
  /*
  * Twiddle
  */
  bool twiddle_;
  std::array<double,2> p_;
  std::array<double,2> dp_;
  double best_err_;
  double current_err_;
  double current_run_err_;
  int while_loop_;
  unsigned int for_loop_;
  bool first_call_;
  int for_loop_continuation_point_;
  double cte_start_;
  unsigned int iters_b4_twiddle_resume_;
  unsigned int non_twiddle_iters_;
  
  /*
  * Debug
  */
  unsigned int total_steps_;
  unsigned int extreme_value_count_;

  /*
  * Constructor
  */
  PID(std::string);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Kd, double Ki, bool twiddle = false);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Calculate steering angle
  */
  double CalculateTargetValue(double cte);
   
  /*
  * Twiddle
  */
  void Twiddle(double cte, double threshold = 0.1);
};

#endif /* PID_H */

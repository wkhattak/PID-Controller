#include "PID.h"
#include <iostream>
#include <numeric>
#include <math.h>

using namespace std;

PID::PID(string pid_id) {
  id_ = pid_id;
  init_ = false;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  cte_ = 0.0;
  prev_cte_ = 0.0;
  integ_cte_ = 0.0;
  diff_cte_ = 0.0;
  Kp_ = 0;
  Ki_ = 0;
  Kd_ = 0;
  p_[0] = 0;
  p_[1] = 0;
  dp_[0] = 1;
  dp_[1] = 1; 
  best_err_ = 0;
  current_err_ = 0;
  current_run_err_ = 0;
  while_loop_ = 0;
  for_loop_ = 0;
  first_call_ = true;
  for_loop_continuation_point_ = 0;
  twiddle_ = false;
  iters_b4_twiddle_resume_ = 50;
  non_twiddle_iters_ = 0;
  total_steps_ = 0;
  extreme_value_count_ = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki, bool twiddle) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  p_[0] = Kp;
  p_[1] = Kd;
  twiddle_ = twiddle;
  init_ = true;
}

void PID::UpdateError(double cte) {
  cte_ = cte;
  diff_cte_ = cte - prev_cte_;
  prev_cte_ = cte;
  integ_cte_ += cte;
}

double PID::TotalError() {
  return 0.0;
}

double PID::CalculateTargetValue(double cte) {
  total_steps_ += 1;
  if (twiddle_) {
    if (non_twiddle_iters_ >= iters_b4_twiddle_resume_/2) current_run_err_ += (cte*cte); 
    
    if (non_twiddle_iters_ > iters_b4_twiddle_resume_) Twiddle(cte, 0.001);
    else non_twiddle_iters_ += 1;
    
    cout << id_ << " PID >>> Twiddle Step: " << total_steps_ << " >>> Kp_: " << Kp_ << ", Ki_: " << Ki_<< ", Kd_: " << Kd_ << endl;
  }
  
  UpdateError(cte);
  double target_value = -Kp_ * cte_ - Kd_ * diff_cte_ - Ki_ * integ_cte_;
  
  // keep target value between -1/+1
  if (target_value > 1) { 
    extreme_value_count_ += 1;
    target_value = 1;
  }
  else if (target_value < -1) {
    extreme_value_count_ += 1;
     target_value = -1;
  }
  cout << id_ << " PID >>> Extreme value count: " << extreme_value_count_ << endl;
  return target_value;
}

void PID::Twiddle(double cte, double threshold) {
  non_twiddle_iters_ = 0;
  if (first_call_) {
    best_err_ = current_run_err_/(iters_b4_twiddle_resume_/2); // average error
    current_run_err_ = 0; // reset
    first_call_ = false;
    goto while_loop;
  }
  else {
    current_err_ = current_run_err_/(iters_b4_twiddle_resume_/2); // average error
    current_run_err_ = 0; // reset
    goto for_loop;
  }
  
  while_loop: {
    double current_total_err = accumulate(begin(dp_), end(dp_), 0.0, plus<double>());
    if (current_total_err > threshold || fabs(cte) > 0.0029) {
      goto for_loop;
    }
    else {
      cout << "Skipping Twiddle loop as (current_total_err > threshold || cte > 0.0029) condition NOT met!!!, Current total error: " << current_total_err << ", CTE: " << cte << endl;
      return;
    }
  }  

  for_loop: {
    if (for_loop_ < p_.size()) {
      //cout << "Twiddle >>> optimizing parameter: " << for_loop_ << ", best error: " << best_err_ << endl;
      if (for_loop_continuation_point_ == 0) {
        p_[for_loop_] += dp_[for_loop_];
        for_loop_continuation_point_ = 1;
        Kp_ = p_[0];
        //cout << "Kp_: " << Kp_ << ", for-loop: 0" << endl;
        Kd_ = p_[1]; 
        return;
      }
      else if (for_loop_continuation_point_ == 1) {
        if (current_err_ < best_err_) { 
          best_err_ = current_err_;
          dp_[for_loop_] *= 1.1; 
          for_loop_continuation_point_ = 0;
          //cout << "Twiddle >>> parameter: " << for_loop_ << " optimized, value: " << p_[for_loop_] << endl;
          for_loop_ += 1;
          //cout << "Current error: " << current_err_ << " better than best error: " << best_err_ << endl;
          goto for_loop; // start for loop again
        }
        else {
          p_[for_loop_] -= 2 * dp_[for_loop_];
          for_loop_continuation_point_ = 2;
          Kp_ = p_[0];
          //cout << "Kp_: " << Kp_ << ", for-loop: 1-Else" << endl;
          Kd_ = p_[1];
          return;
        }
      }
      else if (for_loop_continuation_point_ == 2) {
        if (current_err_ < best_err_) { 
          best_err_ = current_err_;
          dp_[for_loop_] *= 1.1; 
        }
        else {
          p_[for_loop_] += dp_[for_loop_];
          dp_[for_loop_] *= 0.9; 
        }
        for_loop_continuation_point_ = 0;
        //cout << "Twiddle >>> parameter: " << for_loop_ << " optimized, value: " << p_[for_loop_] << endl;
        for_loop_ += 1;
        goto for_loop; // start for loop's next iteration
      }
    }
    else { // reset
      for_loop_ = 0;
      for_loop_continuation_point_ = 0;
      //cout << "Twiddle >>> END - while-loop iteration: " << while_loop_ << ", twiddle total error: " << accumulate(begin(dp_), end(dp_), 0.0, plus<double>()) << ", twiddle threshold: " << threshold << ", >>> Kp_: " << Kp_ << ", Ki_: " << Ki_ << ", Kd_: " << Kd_ << endl;
      while_loop_ += 1;
      goto while_loop; // start while loop's next iteration
    }
  }
}
#include "PID.h"
#include <iostream>
#include <vector>
#include <iostream>
#include <algorithm>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * DONE: Initialize PID coefficients (and errors, if needed)
   */
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * DONE: Update PID errors based on cte.
   */
  
  // use previous p_error for i_error and d_error
  i_error += p_error;
  
  if (is_first_update) {
    d_error = 0.0;
    is_first_update = false;
  }
  else {
    d_error = cte - p_error;
  }
  
  p_error = cte;
}

double PID::TotalError() {
  /**
   * DONE: Calculate and return the total error
   */
  //std::cout << -Kp << "*" << p_error << " - " << Ki << "*" << i_error << " - " << Kd << "*" << d_error << std::endl;
  return std::max(-1.0, std::min(1.0, -Kp * p_error - Ki * i_error - Kd * d_error));
}

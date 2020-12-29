#include "twiddle.h"
#include "pid.h"
#include <iostream>
#include <limits>
#include <cmath>

constexpr double TUNING_MULT = 1.2;

Twiddle::~Twiddle() {}

void Twiddle::Init(PID * pid_, int iter_length_, double limit_cte_fail_, double limit_cte_success_,
    std::vector<double> tuning_params_, std::vector<double> tuning_goals_) { 
  
  std::cout << "Twiddle Init: ";
  std::cout << "Kpid=(" << pid_->Kp << ", " << pid_->Ki << ", " << pid_->Kd
    << "), tune=(" << tuning_params_[0] << ", " << tuning_params_[1] << ", " << tuning_params_[2]
    << "), goals=(" << tuning_goals_[0] << ", " << tuning_goals_[1] << ", " << tuning_goals_[2] << ")";

  start_count = 0;

  Reset();
  
  pid_ptr = pid_;
  iter_length = iter_length_;
  limit_cte_fail = limit_cte_fail_;
  limit_cte_success = limit_cte_success_;
  tuning_params = tuning_params_;
  tuning_goals = tuning_goals_;
  best_error = std::numeric_limits<double>::max();
  param_index = 0;
  state = State::FIRST;  // this causes the next iteration to nudge the value up
}

void Twiddle::Reset() {
  enabled = true;
  iter_count = 0;
  max_abs_cte = 0.0;
  sum_abs_cte = 0.0;
  if (pid_ptr) {
    pid_ptr->p_error = 0.0;
    pid_ptr->i_error = 0.0;
    pid_ptr->d_error = 0.0;
  }
  ++start_count;
  std::cout << "\nRound #" << start_count << ":  " << std::endl;
}

void Twiddle::PrintResults() {
  if (state != State::FINISHED) {
    std::cout << "\nTwiddle ended after " << start_count << " starts. Reason: ";
    if (param_index == -1 || IsGoalReached()) {
      std::cout << "Kp, Ki, Kd goals reached.";
    }
    else if (max_abs_cte <= limit_cte_success) {
      std::cout << "all cte was below limit_cte_success.";
    }
    else if (std::abs(pid_ptr->p_error) >= limit_cte_fail) {
      std::cout << "out of road.";
    }

    std::cout << " Kpid=(" << pid_ptr->Kp << ", " << pid_ptr->Ki << ", " << pid_ptr->Kd;
    std::cout << "), max abs CTE: " << max_abs_cte << ", avg: " << sum_abs_cte / double(iter_count) << std::endl;
    if (best_params[0])
      std::cout << "Best params: Kp = " << best_params[0] << ", Ki = " << best_params[1] << ", Kd = " << best_params[2] << std::endl;
    else
      std::cout << "Best params: -none-" << std::endl;
  }
}

bool Twiddle::IsGoalReached() const {
  return (std::abs(tuning_params[0]) <= tuning_goals[0]
       && std::abs(tuning_params[1]) <= tuning_goals[1]
       && std::abs(tuning_params[2]) <= tuning_goals[2]);
}

double Twiddle::CalculateError() const {
  double avg_cte = sum_abs_cte / double(iter_count);

  // apply weights:
  double result = 1.0 * avg_cte 
                + 9.0 * max_abs_cte;

  std::cout << "err: avg=" << avg_cte << " max=" << max_abs_cte << " weighted=" << result << " " << std::flush;
  return result;
}

int Twiddle::GetNextIndex() const {
  for (int i = param_index + 1; i < param_index + 4; ++i) {
    int imod3 = i % 3;
    if (tuning_params[imod3] < tuning_goals[imod3])
      continue;
    return imod3;
  }
  return -1;
}

double Twiddle::NudgeValue(int index, State state) {
  // select K
  double * k = &(pid_ptr->Kp);
  switch(index) {
    case 1:
      k = &(pid_ptr->Ki);
      break;
    case 2:
      k = &(pid_ptr->Kd);
      break;
  }

  // select action
  switch(state)
  {
    case State::UP:
      *k += tuning_params[index];
      break;
    case State::DOWN:
      *k -= tuning_params[index];
      break;
    default:
      std::cout << "WARNING: invalid case in NudgeValue()" << std::endl;
      break;
  }

  return *k;
}

void Twiddle::PrintState() {
  static std::vector<std::string> state_names{"FIRST", "UP", "DOWN", "FINISHING", "FINISHED"};
  std::cout << state_names[int(state)] << " ";
}

void Twiddle::PrintKs(bool endl) {
  std::cout << " Kpid=(" << pid_ptr->Kp << ", " << pid_ptr->Ki << ", " << pid_ptr->Kd
    << "), tune=(" << tuning_params[0] << ", " << tuning_params[1] << ", " << tuning_params[2]
    << "), best=(" << best_params[0] << ", " << best_params[1] << ", " << best_params[2] << ")";

  if (endl)
    std::cout << std::endl;
}

bool Twiddle::Tune() {
  if (enabled && state != State::FINISHED) {
    if (iter_count < iter_length) {  // round in progress
      ++iter_count;
      double abs_error = std::abs(pid_ptr->p_error);
      max_abs_cte = std::max(max_abs_cte, abs_error);
      sum_abs_cte += abs_error;
      if (abs_error >= limit_cte_fail) {  // out of the road
        PrintResults();
        state = State::FINISHED;
      }
      return false;
    }
    else {  // round ended
      PrintState();
      if (max_abs_cte <= limit_cte_success) {  // iteration was a success, end twiddle
        PrintResults();
        state = State::FINISHED;
      }
      else {  // start new iteration

        switch(state) {

          case State::FIRST: {  // the very first round just ended
            best_error = CalculateError();
            Reset();
            state = State::UP;
            NudgeValue(0, state);
            std::cout << "After FIRST state nudge up Kp by tuning_param: " << tuning_params[0];
            PrintKs(true);
            return true;
          } break;

          case State::UP: {  // last action was an up nudge
            double error = CalculateError();
            if (error < best_error) {  // up worked, go to next param and nudge that up
              tuning_params[param_index] *= TUNING_MULT;
              std::cout << "UP worked, err=" << error << ", best err=" << best_error << ", tuning_params[" << param_index << "] increased";
              best_error = error;
              best_params[0] = pid_ptr->Kp;
              best_params[1] = pid_ptr->Ki;
              best_params[2] = pid_ptr->Kd;
              PrintKs(false);
              // continues below to the next 
            }
            else {  // up did not work, nudge back and down
              Reset();
              state = State::DOWN;
              NudgeValue(param_index, state);
              NudgeValue(param_index, state);
              std::cout << "UP did not work, err=" << error << ", best err=" << best_error << "nudge down twice K[" << param_index << "]";
              PrintKs(true);
              return true;
            }
          } break;
          
          case State::DOWN: {  // last action was a down nudge
            double error = CalculateError();
            if (error < best_error) {  // down worked, go to next param and nudge that up
              tuning_params[param_index] *= TUNING_MULT;
              std::cout << "DOWN worked, err=" << error << ", best err=" << best_error << ", tuning_params[" << param_index << "] increased";
              best_error = error;
              best_params[0] = pid_ptr->Kp;
              best_params[1] = pid_ptr->Ki;
              best_params[2] = pid_ptr->Kd;
              PrintKs(false);
              // continues below to the next 
            }
            else {  // down did not work, reset value, we are close, decrease mult, go to next param and nudge that up
              std::cout << "DOWN did not work, err=" << error << ", best err=" << best_error << " reset K[" << param_index
                << "] to orig, decrease tuning_params[" << param_index << "]";
              NudgeValue(param_index, State::UP);
              tuning_params[param_index] /= TUNING_MULT;
              // continues below to the next 
            }
          } break;

          case State::FINISHING: {
            PrintResults();
            state = State::FINISHED;
            return false;
          } break;

          case State::FINISHED:
            return false;
            break;
        }

        // go to next param and nudge it up
        param_index = GetNextIndex();
        if (param_index == -1) {
          state = State::FINISHING;
          return false; 
        }
        Reset();
        state = State::UP;
        NudgeValue(param_index, state);
        std::cout << "Nudge up K[" << param_index << "]";
        PrintKs(true);
        return true;
      }
    }  // round ended
  }  // twiddle enabled
  else if (!enabled) {
    std::cout << "Twindle disabled." << std::endl;
    return false;
  }
  return false;
}

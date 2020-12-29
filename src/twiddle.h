#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "pid.h"
#include <vector>

class Twiddle {
 public:
  /**
   * Possible states of twiddle
   * FIRST - during the first round
   * UP - trying an up-nudge
   * DOWN - trying a down-nudge
   * FINISHING - transient state to FINISHED, prints the reason of finishing and stats
   * FINISHED - twiddle ended from some reason
   */
  enum class State {FIRST, UP, DOWN, FINISHING, FINISHED};

  Twiddle() = default;
  virtual ~Twiddle();

  /**
   * Prints the name of the last state, eg. UP, DOWN
   */
  void PrintState();

  /**
  * Set up the initial values for twiddle. Call it only once.
  * @param pid PID controller to twiddle. NOTE: It gets modified.
  * @param iter_length Number of frames a single iteration lasts.
  * @param limit_cte_fail The iteration fails if cte reaches the limit.
  * @param limit_cte_success If cte never reaches this, then twiddling is finished.
  * @param tuning_params Kp, Ki, Kd
  * @param tuning_goals Kp, Ki, Kd: Twiddle ends when all these are reached.
  */
  void Init(PID * pid_, int iter_length_, double limit_cte_fail_, double limit_cte_success_,
    std::vector<double> tuning_params_, std::vector<double> tuning_limits_);

  /**
  * Reset before starting a new iteration.
  */
  void Reset();

  /**
   * Sets one single parameter up or down.
   * @param index which K to adjust, 0..2
   * @param state State::UP or State::DOWN
   * @output the modified value
   */
  double NudgeValue(int index, State state);
  
  /**
   * Twiddles the values if iter_length reached. iter_count is increased after every call. 
   * Call it after each TotalError() call.
   * @output true if sim needs to be reset for new iteration
   */
  bool Tune();

 private:
  bool IsGoalReached() const;
  /**
   * Prints the twiddle results when finished.
   */
  void PrintResults();

  /**
   * Print K values, tuning values and tuning goals.
   */
  void PrintKs(bool endl);

  /**
   * Calculates error from different values, e.g. max cte and avg cte.
   */
  double CalculateError() const;

  /**
   * Returns the next parameter index to nudge. 0..1
   * Checks if the parameters have already reached their goals.
   * @output the index of the next param or -1 if all reached its goal
   */
  int GetNextIndex() const;

 private:
  bool enabled = false;

  PID * pid_ptr = nullptr;
  int iter_length;            // how many frames to evaluate before starting new round
  int iter_count;             // how many frames has passed from the round
  double max_abs_cte;         // keep track of the worst CTE during the round
  double sum_abs_cte;         // used for calculating average CTE
  double limit_cte_fail;      // fails if crossed
  double limit_cte_success;   // reached our goal if all CTE was below it
  std::vector<double> tuning_params;  // nudge with these values
  std::vector<double> tuning_goals;   // reached our goal if all tuning_params is below these values
  std::vector<double> best_params{0.0, 0.0, 0.0};  // keep track of params that worked the best
  int param_index;            // which param to nudge next
  double best_error;          // weighted error
  State state;                // the current state of the twiddle FSM
  int start_count;            // how many rounds (iterations) have been started
};

#endif // TWIDDLE_H

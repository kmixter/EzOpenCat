#ifndef _AUTO_MODE_H
#define _AUTO_MODE_H

class ServoAnimator;
class PRNG;

enum AutoModeState {
  kStateSleeping,
  kStateStretch,
  kStateBalance,
  kStateSit,
  kStateWalkInPlace,
  kStateCount
};

class AutoMode {
 public:
  struct StateData {
    char next_state_prob[kStateCount];
    char seconds_exit_median;
    char seconds_exit_variance;
    char ms_per_degree;
    int animation_sequence;
  };

  void Initialize(ServoAnimator* animator, PRNG* prng);
  void SetEnabled(bool enabled);
  void Update(unsigned long millis);
  bool enabled() const { return enabled_; }
  AutoModeState GetState() const {
    return state_;
  }

#ifdef TESTING
  void SetStateData(StateData* data) {
    state_data_ = data;
  }
#endif  // TESTING

 private:

  ServoAnimator* servo_animator_ = nullptr;
  PRNG* prng_ = nullptr;
  int saved_ms_per_degree_ = 0;
  bool enabled_ = false;
  AutoModeState state_ = kStateSleeping;
  unsigned long millis_next_state_ = 0;
  StateData* state_data_;
};

#endif  // _AUTO_MODE_H
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
    bool look_around;
    int animation_sequence;
  };

  void Initialize(ServoAnimator* animator, PRNG* prng);
  void SetEnabled(bool enabled);
  void Update(unsigned long millis);
  bool enabled() const { return enabled_; }
  AutoModeState GetState() const {
    return state_;
  }
  void SetLookAroundEnabled(bool enabled) {
    look_around_enabled_ = enabled;
  }

#ifdef TESTING
  void SetStateData(StateData* data) {
    state_data_ = data;
  }
#endif  // TESTING

 private:
  void LookAround(unsigned long millis_now);

  ServoAnimator* servo_animator_ = nullptr;
  PRNG* prng_ = nullptr;
  int saved_ms_per_degree_ = 0;
  bool enabled_ = false;
  AutoModeState state_ = kStateSleeping;
  unsigned long millis_next_state_ = 0;
  StateData* state_data_;

  unsigned long millis_next_look_around_ = 0;
  bool look_around_enabled_ = true;
};

#endif  // _AUTO_MODE_H
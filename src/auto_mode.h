#ifndef _AUTO_MODE_H
#define _AUTO_MODE_H

class ServoAnimator;
class PRNG;

class AutoMode {
 public:
  void Initialize(ServoAnimator* animator, PRNG* prng) {
    servo_animator_ = animator;
    prng_ = prng;
  }
  void SetEnabled(bool enabled);
  void Update(unsigned long millis);
  bool enabled() const { return enabled_; }

 private:
  enum State {
    Init,
    Stretch,
    Wake,
    WalkInPlace,
    WakeToRest,
    FallAsleep
  } state_ = Init;

  ServoAnimator* servo_animator_ = nullptr;
  PRNG* prng_ = nullptr;
  int saved_ms_per_degree_ = 0;
  bool enabled_ = false;
  unsigned long last_animation_time_ = 0;
  unsigned long next_animation_delay_ = 0;
};

#endif  // _AUTO_MODE_H
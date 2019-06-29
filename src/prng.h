#ifndef _PRNG_H
#define _PRNG_H

#include <stdint.h>

// PseudoRandomNumberGenerator is just too long to write out many times.
class PRNG {
 public:
  PRNG() {}
  virtual uint32_t Get() = 0;
  uint32_t Roll(uint32_t max) {
    return Get() % max;
  }
};

// From http://burtleburtle.net/bob/rand/smallprng.html.
class SmallPRNG : public PRNG {
 public:
  typedef unsigned long int u4;
  typedef struct ranctx { u4 a; u4 b; u4 c; u4 d; } ranctx;

  SmallPRNG(uint32_t seed) { SetSeed(seed); }
  void SetSeed(uint32_t seed);
  uint32_t Get() override;

 private:
  ranctx context_;
};

#ifdef TESTING
class NotAtAllRandom : public PRNG {
 public:
  NotAtAllRandom(uint32_t value) : value_(value) {}
  uint32_t Get() override {
    return value_;
  }

 private:
  uint32_t value_;
};

#include <list>

class RandomSequenceFake : public PRNG {
 public:
  RandomSequenceFake() {}
  uint32_t Get() override {
    if (sequence_->size() == 0) *((int*)nullptr) = 0;
    uint32_t result = sequence_->front();
    sequence_->pop_front();
    return result;
  }

  void SetSequence(std::list<uint32_t>* sequence) {
    sequence_ = sequence;
  }

 private:
  std::list<uint32_t>* sequence_;
};
#endif

#endif  // _PRNG_H
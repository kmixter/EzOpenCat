#include "prng.h"

#include <memory>
#include <gtest/gtest.h>

using std::unique_ptr;

class SmallPRNGTest : public testing::Test {
 protected:
  void SetUp() override {
    prng_.reset(new SmallPRNG(1));
  }

  unique_ptr<SmallPRNG> prng_;
};

TEST_F(SmallPRNGTest, Initialize) {
}

TEST_F(SmallPRNGTest, SpreadIn10000) {
  bool x[10000] = {false};
  for (int i = 0; i < 10000; ++i) {
    uint32_t num = prng_->Get();
    x[num % 10000] = true;
  }
  int hit = 0;
  for (int i = 0; i < 10000; ++i) {
    if (x[i]) ++hit;
  }
  printf("SmallPRNG Spread=%d/10000\n", hit);
  EXPECT_GT(hit, 5000);
}

TEST_F(SmallPRNGTest, Roll) {
  bool x[7] = { false };
  int reached = 0;

  for (int i = 0; i < 100; ++i) {
    uint32_t num = prng_->Roll(7);
    ASSERT_LT(num, 7U);
    if (!x[num]) {
      x[num] = true;
      ++reached;
    }
  }
  EXPECT_EQ(7, reached);
}

TEST_F(SmallPRNGTest, RandSpread) {
  bool x[10000] = {false};
  srand(1);
  for (int i = 0; i < 10000; ++i) {
    uint32_t num = rand();
    x[num % 10000] = true;
  }
  int hit = 0;
  for (int i = 0; i < 10000; ++i) {
    if (x[i]) ++hit;
  }
  printf("rand() Spread=%d\n", hit);
  EXPECT_GT(hit, 5000);
}

TEST_F(SmallPRNGTest, Deterministic) {
  uint32_t seeds[100][100];
  for (int seed = 0; seed < 100; ++seed) {
    prng_->SetSeed(seed);
    int previous_matches = 0;
    for (int seq = 0; seq < 100; ++seq) {
      seeds[seed][seq] = prng_->Get();
      if (seed > 0 && seeds[seed - 1][seq] == seeds[seed][seq])
        previous_matches++;
    }
    EXPECT_LT(previous_matches, 100);
  }

  for (int seed = 0; seed < 100; ++seed) {
    prng_->SetSeed(seed);
    for (int seq = 0; seq < 100; ++seq)
      EXPECT_EQ(seeds[seed][seq], prng_->Get());
  }
}

class NotAtAllRandomTest : public testing::Test {
 protected:
  void SetUp() override {
    prng_.reset(new NotAtAllRandom(1));
  }

  unique_ptr<NotAtAllRandom> prng_;
};

TEST_F(NotAtAllRandomTest, Always) {
  for (int i = 0; i < 100; ++i)
    ASSERT_EQ(1U, prng_->Get());
}
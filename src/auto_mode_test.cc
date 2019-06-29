#include "auto_mode.h"

#include <gtest/gtest.h>

#include "eeprom_settings.h"
#include "prng.h"
#include "servo_animator_testfake.h"

class AutoModeTest : public testing::Test {
 protected:
  AutoModeTest() {}

  void SetUp() override {
    auto_mode_.Initialize(&animator_, &prng_);
    auto_mode_.SetLookAroundEnabled(false);
    prng_.SetSequence(&prng_list_);
  }

  AutoMode auto_mode_;
  RandomSequenceFake prng_;
  std::list<uint32_t> prng_list_ = { 0, 0, 0, 0 };
  ServoAnimatorTestFake animator_;
};

TEST_F(AutoModeTest, Initialize) {
  EXPECT_FALSE(auto_mode_.enabled());
}

TEST_F(AutoModeTest, Enabled) {
  auto_mode_.SetEnabled(true);
  EXPECT_TRUE(auto_mode_.enabled());
}

TEST_F(AutoModeTest, DisabledStart) {
  auto_mode_.SetEnabled(false);
  EXPECT_FALSE(auto_mode_.enabled());
}

TEST_F(AutoModeTest, EnabledThenDisabled) {
  auto_mode_.SetEnabled(true);
  EXPECT_TRUE(auto_mode_.enabled());
  auto_mode_.SetEnabled(false);
  EXPECT_FALSE(auto_mode_.enabled());
}

TEST_F(AutoModeTest, EnableDisableRestoresSpeed) {
  animator_.set_ms_per_degree(50);
  ASSERT_EQ(50, animator_.ms_per_degree());
  auto_mode_.SetEnabled(true);
  animator_.set_ms_per_degree(100);
  auto_mode_.Update(1);
  ASSERT_NE(100, animator_.ms_per_degree());
  auto_mode_.SetEnabled(false);
  EXPECT_EQ(50, animator_.ms_per_degree());
}

TEST_F(AutoModeTest, SleepingStateTransitionToStretch) {
  prng_list_ = { 49, 0 };
  auto_mode_.SetEnabled(true);
  ASSERT_EQ(kStateSleeping, auto_mode_.GetState());
  auto_mode_.Update(1);
  ASSERT_EQ(kStateStretch, auto_mode_.GetState());
}

TEST_F(AutoModeTest, SleepingStateTransitionToBalance) {
  prng_list_ = { 60, 0 };
  auto_mode_.SetEnabled(true);
  auto_mode_.Update(1);
  ASSERT_EQ(kStateBalance, auto_mode_.GetState());
}

TEST_F(AutoModeTest, SequentialTransitions) {
  auto_mode_.SetEnabled(true);
  ASSERT_EQ(kStateSleeping, auto_mode_.GetState());
  prng_list_ = { 40, 1 };

  auto_mode_.Update(0);

  ASSERT_EQ(kStateStretch, auto_mode_.GetState());
  ASSERT_TRUE(animator_.animating());
  ASSERT_TRUE(auto_mode_.enabled());
  ASSERT_EQ(kAnimationStretch, animator_.animation_sequence());
  EXPECT_EQ(10, animator_.ms_per_degree());

  animator_.set_animating(false);
  auto_mode_.Update(1999);

  ASSERT_EQ(kStateStretch, auto_mode_.GetState());
  ASSERT_FALSE(animator_.animating());
  ASSERT_EQ(kAnimationStretch, animator_.animation_sequence());

  prng_list_ = { 0, 5 };
  auto_mode_.Update(2000);
  ASSERT_EQ(kStateBalance, auto_mode_.GetState());
  ASSERT_EQ(kAnimationBalance, animator_.animation_sequence());
  ASSERT_TRUE(animator_.animating());

  animator_.set_animating(false);
  auto_mode_.Update(11999);
  ASSERT_EQ(kStateBalance, auto_mode_.GetState());
  ASSERT_FALSE(animator_.animating());

  prng_list_ = { 30, 0 };
  animator_.set_animating(false);
  auto_mode_.Update(12000);
  ASSERT_TRUE(animator_.animating());
  ASSERT_EQ(kStateSit, auto_mode_.GetState());
  ASSERT_EQ(kAnimationSit, animator_.animation_sequence());

  animator_.set_animating(false);
  auto_mode_.Update(16999);
  ASSERT_FALSE(animator_.animating());

  prng_list_ = { 30, 0 };
  animator_.set_animating(false);
  auto_mode_.Update(17000);
  ASSERT_TRUE(animator_.animating());
  ASSERT_EQ(kStateBalance, auto_mode_.GetState());

  ASSERT_TRUE(auto_mode_.enabled());
}
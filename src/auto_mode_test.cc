#include "auto_mode.h"

#include <gtest/gtest.h>

#include "eeprom_settings.h"
#include "prng.h"
#include "servo_animator_testfake.h"

static AutoMode::StateData s_state_data[kStateCount] = {
  {
    { 0, 50, 20, 20, 10, 0 },
    0, 0,
    10,
    kAnimationRest,
  },
  {
    { 0, 0, 40, 40, 20, 0  },  // StateStretch
    2, 1,
    10,
    kAnimationStretch
  },
  {
    { 0, 0, 0, 40, 30, 30 },  // StateBalance
    10, 5,  // StateBalance
    4,
    kAnimationBalance,
  },
  {
    { 0, 0, 60, 0, 10, 30 },  // StateSit
    10, 5, // StateSit
    4,
    kAnimationSit,
  },
  {
    { 0, 0, 30, 40, 0, 30 },  // StateWalkInPlace
    2, 1,  // StateWalkInPlace
    2,
    kAnimationWalkInPlace,
  },
  {
    { 0, 40, 20, 20, 0, 20 },  // StateSleeping
    20, 4,  // StateSleeping
    10,
    kAnimationRest
  }
};

class AutoModeTest : public testing::Test {
 protected:
  AutoModeTest() {}

  void SetUp() override {
    auto_mode_.Initialize(&animator_, &prng_);
    auto_mode_.SetStateData(s_state_data);
    prng_.SetSequence(&prng_list_);
  }

  AutoMode auto_mode_;
  RandomSequenceFake prng_;
  std::list<uint32_t> prng_list_ = { 0, 0 };
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

TEST_F(AutoModeTest, InitStateTransitionToStretch) {
  prng_list_ = { 49, 0 };
  auto_mode_.SetEnabled(true);
  ASSERT_EQ(kStateInit, auto_mode_.GetState());
  auto_mode_.Update(1);
  ASSERT_EQ(kStateStretch, auto_mode_.GetState());
}

TEST_F(AutoModeTest, InitStateTransitionToBalance) {
  prng_list_ = { 50, 0 };
  auto_mode_.SetEnabled(true);
  auto_mode_.Update(1);
  ASSERT_EQ(kStateBalance, auto_mode_.GetState());
}

TEST_F(AutoModeTest, InitStateTransitionToSit) {
  prng_list_ = { 70, 0 };
  auto_mode_.SetEnabled(true);
  auto_mode_.Update(1);
  ASSERT_EQ(kStateSit, auto_mode_.GetState());
}

TEST_F(AutoModeTest, InitStateTransitionToWalkInPlace) {
  prng_list_ = { 90, 0 };
  auto_mode_.SetEnabled(true);
  auto_mode_.Update(1);
  ASSERT_EQ(kStateWalkInPlace, auto_mode_.GetState());
}

TEST_F(AutoModeTest, SequentialTransitions) {
  auto_mode_.SetEnabled(true);
  ASSERT_EQ(kStateInit, auto_mode_.GetState());
  prng_list_ = { 35, 1 };

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

  prng_list_ = { 0, 0 };
  animator_.set_animating(false);
  auto_mode_.Update(12000);
  ASSERT_TRUE(animator_.animating());
  ASSERT_EQ(kStateSit, auto_mode_.GetState());
  ASSERT_EQ(kAnimationSit, animator_.animation_sequence());

  animator_.set_animating(false);
  auto_mode_.Update(16999);
  ASSERT_FALSE(animator_.animating());

  prng_list_ = { 0, 0 };
  animator_.set_animating(false);
  auto_mode_.Update(17000);
  ASSERT_TRUE(animator_.animating());
  ASSERT_EQ(kStateBalance, auto_mode_.GetState());

  ASSERT_TRUE(auto_mode_.enabled());
}
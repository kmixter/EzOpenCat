#include "auto_mode.h"

#include <gtest/gtest.h>

#include "eeprom_settings.h"
#include "servo_animator_testfake.h"

class AutoModeTest : public testing::Test {
 protected:
  AutoModeTest() {}

  void SetUp() override {
    auto_mode_.Initialize(&animator_);
  }

  AutoMode auto_mode_;
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

TEST_F(AutoModeTest, TransitionsFromRestToBalanceToStretch) {
  auto_mode_.SetEnabled(true);
  auto_mode_.Update(1);
  ASSERT_TRUE(animator_.animating());
  ASSERT_TRUE(auto_mode_.enabled());
  ASSERT_EQ(kAnimationBalance, animator_.animation_sequence());
  EXPECT_EQ(10, animator_.ms_per_degree());

  animator_.set_animating(false);

  // At 3000ms we are just about switch to stretch, but not yet.
  auto_mode_.Update(1000);
  ASSERT_FALSE(animator_.animating());
  ASSERT_TRUE(auto_mode_.enabled());
  ASSERT_EQ(kAnimationBalance, animator_.animation_sequence());

  auto_mode_.Update(1001);
  ASSERT_TRUE(animator_.animating());
  ASSERT_TRUE(auto_mode_.enabled());
  ASSERT_EQ(kAnimationStretch, animator_.animation_sequence());

  animator_.set_animating(false);
  auto_mode_.Update(3001);
  ASSERT_TRUE(auto_mode_.enabled());
  ASSERT_EQ(kAnimationBalance, animator_.animation_sequence());
}
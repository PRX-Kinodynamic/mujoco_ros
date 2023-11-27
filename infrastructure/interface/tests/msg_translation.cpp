#include <gtest/gtest.h>
#include <ros/init.h>

#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_mushr.hpp>

#include <interface/mushr_translation.hpp>

TEST(TranslateMsg, test_mushr_plan_from_ml4kp_bridge_plan)
{
  prx_models::MushrPlan mushr_plan;
  ml4kp_bridge::Plan plan;

  const std::size_t plan_size{ 2 };
  const std::size_t u_dim{ 2 };
  plan.steps.resize(2);

  plan.steps[0].control.point.resize(u_dim);
  plan.steps[1].control.point.resize(u_dim);

  plan.steps[0].control.point[0].data = 1.0;
  plan.steps[0].control.point[1].data = 1.5;
  plan.steps[1].control.point[0].data = 2.0;
  plan.steps[1].control.point[1].data = 2.5;

  interface::translate_msg(mushr_plan, plan);

  ASSERT_DOUBLE_EQ(1.0, mushr_plan.controls[0].steering_angle.data);
  ASSERT_DOUBLE_EQ(1.5, mushr_plan.controls[0].velocity.data);
  ASSERT_DOUBLE_EQ(2.0, mushr_plan.controls[1].steering_angle.data);
  ASSERT_DOUBLE_EQ(2.5, mushr_plan.controls[1].velocity.data);
};

int main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
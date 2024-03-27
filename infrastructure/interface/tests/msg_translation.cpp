#include <gtest/gtest.h>
#include <ros/init.h>

#include <ml4kp_bridge/defs.h>
#include <prx_models/mj_mushr.hpp>

#include <interface/mushr_translation.hpp>

TEST(TranslateMsg, test_mushr_plan_from_ml4kp_bridge_plan)
{
  using prx_models::mushr_t::control::steering_idx;
  using prx_models::mushr_t::control::velocity_idx;

  prx_models::MushrPlan mushr_plan;
  ml4kp_bridge::PlanStamped stamped_plan;

  const std::size_t plan_size{ 2 };
  const std::size_t u_dim{ 2 };
  stamped_plan.plan.steps.resize(2);

  stamped_plan.plan.steps[0].control.point.resize(u_dim);
  stamped_plan.plan.steps[1].control.point.resize(u_dim);

  stamped_plan.plan.steps[0].control.point[velocity_idx].data = 1.0;
  stamped_plan.plan.steps[0].control.point[steering_idx].data = 1.5;
  stamped_plan.plan.steps[1].control.point[velocity_idx].data = 2.0;
  stamped_plan.plan.steps[1].control.point[steering_idx].data = 2.5;

  interface::translate_msg(mushr_plan, stamped_plan);

  ASSERT_DOUBLE_EQ(1.5, mushr_plan.controls[0].steering_angle.data);
  ASSERT_DOUBLE_EQ(1.0, mushr_plan.controls[0].velocity.data);
  ASSERT_DOUBLE_EQ(2.5, mushr_plan.controls[1].steering_angle.data);
  ASSERT_DOUBLE_EQ(2.0, mushr_plan.controls[1].velocity.data);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "msg_translation");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
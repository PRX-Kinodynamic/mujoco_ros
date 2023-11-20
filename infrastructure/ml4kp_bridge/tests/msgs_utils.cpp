#include <gtest/gtest.h>
#include <ros/init.h>

#include <ml4kp_bridge/msgs_utils.hpp>
#include <ml4kp_bridge/plan_step_bridge.hpp>
#include <ml4kp_bridge/plan_bridge.hpp>
#include <ml4kp_bridge/space_bridge.hpp>
#include <ml4kp_bridge/trajectory_bridge.hpp>

#include <ml4kp_bridge/PlanStamped.h>
#include <ml4kp_bridge/PlanStepStamped.h>
#include <ml4kp_bridge/SpacePointStamped.h>
#include <ml4kp_bridge/TrajectoryStamped.h>

namespace mock
{
struct space3d_t
{
  space3d_t() : _x(0), _y(0), _theta(0), _address({ &_x, &_y, &_theta }), _space("EER", _address, "space_test")
  {
  }
  double _x, _y, _theta;
  std::vector<double*> _address;
  prx::space_t _space;
};
}  // namespace mock

TEST(SpaceBridge, test_copy_msg_from_state)
{
  mock::space3d_t test;
  prx::space_t& space{ test._space };

  prx::space_point_t state{ space.make_point() };
  ml4kp_bridge::SpacePointStamped msg;

  space.copy(state, { 1.0, 2.0, 3.1415 });

  ml4kp_bridge::copy(msg, state);

  ASSERT_DOUBLE_EQ(1.0, msg.space_point.state[0].data);
  ASSERT_DOUBLE_EQ(2.0, msg.space_point.state[1].data);
  ASSERT_DOUBLE_EQ(3.1415, msg.space_point.state[2].data);
  ASSERT_LT(0, msg.header.seq);
  ASSERT_LT(msg.header.stamp, ros::Time::now());  // stamp <= now
};

TEST(SpaceBridge, test_copy_state_from_msg)
{
  mock::space3d_t test;
  prx::space_t& space{ test._space };

  prx::space_point_t state{ space.make_point() };
  ml4kp_bridge::SpacePointStamped msg;

  msg.header.seq = 10;
  msg.header.stamp = ros::Time::now();
  msg.space_point.state.resize(3);
  msg.space_point.state[0].data = 1;
  msg.space_point.state[1].data = 2;
  msg.space_point.state[2].data = 3.1415;
  ml4kp_bridge::copy(state, msg);

  ASSERT_DOUBLE_EQ(1.0, state->at(0));
  ASSERT_DOUBLE_EQ(2.0, state->at(1));
  ASSERT_DOUBLE_EQ(3.1415, state->at(2));
};

TEST(TrajectoryBridge, test_copy_msg_from_trajectory)
{
  mock::space3d_t test;
  prx::space_t& space{ test._space };

  prx::space_point_t state{ space.make_point() };
  prx::trajectory_t trajectory{ &space };
  ml4kp_bridge::TrajectoryStamped msg;

  space.copy(state, { 1.0, 1.0, 1.0 });
  trajectory.copy_onto_back(state);

  space.copy(state, { 2.0, 2.0, 2.0 });
  trajectory.copy_onto_back(state);

  ml4kp_bridge::copy(msg, trajectory);

  ASSERT_DOUBLE_EQ(1.0, msg.trajectory.data[0].state[0].data);
  ASSERT_DOUBLE_EQ(1.0, msg.trajectory.data[0].state[1].data);
  ASSERT_DOUBLE_EQ(1.0, msg.trajectory.data[0].state[2].data);

  ASSERT_DOUBLE_EQ(2.0, msg.trajectory.data[1].state[0].data);
  ASSERT_DOUBLE_EQ(2.0, msg.trajectory.data[1].state[1].data);
  ASSERT_DOUBLE_EQ(2.0, msg.trajectory.data[1].state[2].data);

  ASSERT_LT(0, msg.header.seq);
  ASSERT_LT(msg.header.stamp, ros::Time::now());  // stamp <= now
};

TEST(TrajectoryBridge, test_copy_trajectory_from_msg)
{
  mock::space3d_t test;
  prx::space_t& space{ test._space };

  prx::space_point_t state{ space.make_point() };
  prx::trajectory_t trajectory{ &space };
  ml4kp_bridge::TrajectoryStamped msg;

  ml4kp_bridge::SpacePoint pt0{};
  ml4kp_bridge::SpacePoint pt1{};
  pt0.state.resize(3);
  pt1.state.resize(3);
  pt0.state[0].data = 1;
  pt0.state[1].data = 1;
  pt0.state[2].data = 1;
  pt1.state[0].data = 2;
  pt1.state[1].data = 2;
  pt1.state[2].data = 2;
  msg.trajectory.data.push_back(pt0);
  msg.trajectory.data.push_back(pt1);

  ml4kp_bridge::copy(trajectory, msg);

  unsigned state_idx{ 0 };
  ASSERT_DOUBLE_EQ(2, trajectory.size());
  ASSERT_DOUBLE_EQ(1.0, trajectory[state_idx]->at(0));
  ASSERT_DOUBLE_EQ(1.0, trajectory[state_idx]->at(1));
  ASSERT_DOUBLE_EQ(1.0, trajectory[state_idx]->at(2));

  state_idx++;
  ASSERT_DOUBLE_EQ(2.0, trajectory[state_idx]->at(0));
  ASSERT_DOUBLE_EQ(2.0, trajectory[state_idx]->at(1));
  ASSERT_DOUBLE_EQ(2.0, trajectory[state_idx]->at(2));
};

TEST(PlanStepBridge, test_copy_msg_from_plan_step)
{
  mock::space3d_t test;
  prx::space_t& space{ test._space };

  prx::space_point_t ctrl_pt{ space.make_point() };
  space.copy(ctrl_pt, { 1.0, 2.0, 3.0 });
  const double duration{ 0.5 };

  prx::plan_step_t plan_step{ ctrl_pt, duration };
  ml4kp_bridge::PlanStepStamped msg;

  ml4kp_bridge::copy(msg, plan_step);

  ASSERT_DOUBLE_EQ(msg.plan_step.control.state[0].data, 1.0);
  ASSERT_DOUBLE_EQ(msg.plan_step.control.state[1].data, 2.0);
  ASSERT_DOUBLE_EQ(msg.plan_step.control.state[2].data, 3.0);
  ASSERT_LT(msg.header.stamp, ros::Time::now());  // stamp <= now
};

TEST(PlanStepBridge, test_copy_plan_step_from_msg)
{
  mock::space3d_t test;
  prx::space_t& space{ test._space };

  prx::space_point_t ctrl_pt{ space.make_point() };
  space.copy(ctrl_pt, { 0.0, 0.0, 0.0 });
  const double duration{ 0.0 };

  prx::plan_step_t plan_step{ ctrl_pt, duration };
  ml4kp_bridge::PlanStepStamped msg;

  msg.plan_step.control.state.resize(3);
  msg.plan_step.control.state[0].data = 1.0;
  msg.plan_step.control.state[1].data = 2.0;
  msg.plan_step.control.state[2].data = 3.0;

  ml4kp_bridge::copy(plan_step, msg);

  ASSERT_DOUBLE_EQ(plan_step.control->at(0), 1.0);
  ASSERT_DOUBLE_EQ(plan_step.control->at(1), 2.0);
  ASSERT_DOUBLE_EQ(plan_step.control->at(2), 3.0);
  ASSERT_LT(msg.header.stamp, ros::Time::now());  // stamp <= now
};

TEST(PlanBridge, test_copy_msg_from_plan)
{
  mock::space3d_t test;
  prx::space_t& space{ test._space };

  prx::space_point_t ctrl_pt_0{ space.make_point() };
  prx::space_point_t ctrl_pt_1{ space.make_point() };
  space.copy(ctrl_pt_0, { 1.0, 1.0, 1.0 });
  space.copy(ctrl_pt_1, { 2.0, 2.0, 2.0 });
  const std::vector<double> durations{ { 0.5, 1.0 } };

  prx::plan_t plan{ &space };
  plan.copy_onto_back(ctrl_pt_0, durations[0]);
  plan.copy_onto_back(ctrl_pt_1, durations[1]);

  ml4kp_bridge::PlanStamped msg;

  ml4kp_bridge::copy(msg, plan);

  ASSERT_DOUBLE_EQ(msg.plan.steps[0].control.state[0].data, 1.0);
  ASSERT_DOUBLE_EQ(msg.plan.steps[0].control.state[1].data, 1.0);
  ASSERT_DOUBLE_EQ(msg.plan.steps[0].control.state[2].data, 1.0);
  ASSERT_DOUBLE_EQ(msg.plan.steps[1].control.state[0].data, 2.0);
  ASSERT_DOUBLE_EQ(msg.plan.steps[1].control.state[1].data, 2.0);
  ASSERT_DOUBLE_EQ(msg.plan.steps[1].control.state[2].data, 2.0);
  ASSERT_LT(msg.header.stamp, ros::Time::now());  // stamp <= now
};

TEST(PlanBridge, test_copy_plan_from_msg)
{
  mock::space3d_t test;
  prx::space_t& space{ test._space };

  prx::plan_t plan{ &space };
  ml4kp_bridge::PlanStamped msg;

  msg.plan.steps.resize(2);
  msg.plan.steps[0].control.state.resize(3);
  msg.plan.steps[1].control.state.resize(3);
  msg.plan.steps[0].control.state[0].data = 1.0;
  msg.plan.steps[0].control.state[1].data = 1.0;
  msg.plan.steps[0].control.state[2].data = 1.0;
  msg.plan.steps[1].control.state[0].data = 2.0;
  msg.plan.steps[1].control.state[1].data = 2.0;
  msg.plan.steps[1].control.state[2].data = 2.0;

  ml4kp_bridge::copy(plan, msg);

  unsigned idx{ 0 };
  ASSERT_DOUBLE_EQ(plan[idx].control->at(0), 1.0);
  ASSERT_DOUBLE_EQ(plan[idx].control->at(1), 1.0);
  ASSERT_DOUBLE_EQ(plan[idx].control->at(2), 1.0);

  idx++;
  ASSERT_DOUBLE_EQ(plan[idx].control->at(0), 2.0);
  ASSERT_DOUBLE_EQ(plan[idx].control->at(1), 2.0);
  ASSERT_DOUBLE_EQ(plan[idx].control->at(2), 2.0);
  ASSERT_LT(msg.header.stamp, ros::Time::now());  // stamp <= now
};

int main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
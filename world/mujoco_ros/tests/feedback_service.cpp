#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>

#include <prx_models/mj_mushr.hpp>
#include <mujoco_ros/simulator.hpp>
#include <mujoco_ros/feedback_service.hpp>

std::shared_ptr<ros::NodeHandle> nh;

TEST(TestFeedbackService, call_service)
{
  using FeedbackClient = mj_ros::feedback_client_t<prx_models::MushrFeedback>;
  const std::string model_path{ ros::package::getPath("prx_models") + "/models/mushr/mushr.xml" };
  std::cout << "Model path: " << model_path << std::endl;
  mj_ros::SimulatorPtr sim{ mj_ros::simulator_t::initialize(model_path, false) };
  FeedbackClient feedback_client(*nh, sim, 10);

  EXPECT_TRUE(feedback_client.ok());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feedback_client_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <ros/init.h>
#include <gtest/gtest.h>
#include <nodelet/nodelet.h>
#include <utils/nodelet_as_node.hpp>

namespace mock
{
template <typename Base>
class nodelet_t : public Base
{
public:
  nodelet_t(){};
  virtual void onInit(){};
  bool ok()
  {
    return true;
  }
};
}  // namespace mock

TEST(NodeletAsNode, nodelet_as_nodelet)
{
  mock::nodelet_t<nodelet::Nodelet> nodelet_obj{};
  EXPECT_TRUE(nodelet_obj.ok());
}

TEST(NodeletAsNode, nodelet_as_node)
{
  mock::nodelet_t<utils::nodelet_as_node_t> node_obj{};
  EXPECT_TRUE(node_obj.ok());
}

int main(int argc, char** argv)
{
  // ros::Time::init();
  ros::init(argc, argv, "nodelet_as_node_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
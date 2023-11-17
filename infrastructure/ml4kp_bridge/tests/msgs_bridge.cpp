#include <gtest/gtest.h>
#include <ros/init.h>
#include <ml4kp_bridge/space_bridge.hpp>

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

TEST(SpaceBridge, test_copy_state_to_msg)
{
  mock::space3d_t test;
  prx::space_t& space{ test._space };

  // prx::space_point_t state{ space.make_point() };
  // ml4kp_bridge::SpacePoint msg;

  // (*state)[0] = 1;
  // (*state)[1] = 2;
  // (*state)[2] = 3.1415;

  // ml4kp_bridge::copy(msg, *state);

  // ASSERT_DOUBLE_EQ((*state)[0], msg.state[0].data);
  // ASSERT_DOUBLE_EQ((*state)[1], msg.state[1].data);
  // ASSERT_DOUBLE_EQ((*state)[2], msg.state[2].data);
};

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
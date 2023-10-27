#include <gtest/gtest.h>
#include <prx/utilities/general/constants.hpp>
#include <ros/init.h>

TEST(TestConstants, testConstants)
{
  ASSERT_NE(prx::lib_path, std::string(""));  // This should fail if the env variable is not properly set
  ASSERT_DOUBLE_EQ(prx::constants::pi, 3.1415926535897932385);
  ASSERT_DOUBLE_EQ(prx::constants::epsilon, 1e-7);
  ASSERT_DOUBLE_EQ(prx::constants::infinity, std::numeric_limits<double>::infinity());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
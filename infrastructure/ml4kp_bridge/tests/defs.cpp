#include <ros/init.h>
#include <gtest/gtest.h>

#include <ml4kp_bridge/defs.h>

TEST(TestSystemFactory, testAvailableSystems)
{
  const std::vector<std::string> available_systems{ prx::system_factory_t::available_systems() };

  // Checking that there *are* systems
  ASSERT_TRUE(available_systems.size() > 0);
  for (auto name : available_systems)
  {
    ASSERT_TRUE(name.size() > 0);  // Names are not-empty
  }
}

TEST(TestSystemFactory, testCreateSystem)
{
  const std::vector<std::string> available_systems{ prx::system_factory_t::available_systems() };
  for (auto name : available_systems)
  {
    const std::string path{ name + "_path" };
    prx::system_ptr_t sys_ptr{ prx::system_factory_t::create_system(name, path) };
    ASSERT_TRUE(sys_ptr != nullptr);
    ASSERT_TRUE(sys_ptr->get_pathname() == path);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
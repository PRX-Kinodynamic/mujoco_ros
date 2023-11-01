#pragma once

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <strstream>
#include <iomanip>
#include "GLFW/glfw3.h"
#include "ros/ros.h"
#include "mujoco/mujoco.h"

namespace mj_ros
{
namespace constants
{
static double frequency{ 50.0 };
}
}  // namespace mj_ros
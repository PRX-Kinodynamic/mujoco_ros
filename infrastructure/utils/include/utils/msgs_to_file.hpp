#pragma once
#include <fstream>
#include <ackermann_msgs/AckermannDriveStamped.h>

namespace utils
{

inline void to_file(const ackermann_msgs::AckermannDriveStamped& msg, std::ofstream& ofs)
{
  static int ackermann_msgs_idx{ 0 };
  if (ackermann_msgs_idx == 0)
  {
    ofs << "# steering_angle speed acceleration stamp ";
  }
  ofs << msg.drive.steering_angle << " ";
  ofs << msg.drive.speed << " ";
  ofs << msg.drive.acceleration << " ";
  ofs << msg.header.stamp << "\n";
  ackermann_msgs_idx++;
}

}  // namespace utils

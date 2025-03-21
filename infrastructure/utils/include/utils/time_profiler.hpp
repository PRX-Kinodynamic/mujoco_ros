#include <fstream>
#include <ros/ros.h>

namespace utils
{

class time_profiler_t
{
public:
  time_profiler_t()
  {
  }

  time_profiler_t(const std::string filename) : _ofs(filename)
  {
  }

  void set_filename(const std::string filename)
  {
    _ofs.open(filename);
  }

  void start()
  {
    _start = ros::WallTime::now();
    _started = true;
  }

  ros::WallDuration checkpoint(const std::string msg = "")
  {
    const ros::WallTime now{ ros::WallTime::now() };
    const ros::WallDuration dt{ now - _start };
    if (msg != "")
    {
      _ofs << msg << " ";
    }
    _ofs << dt << " ";
    return dt;
  }

  ros::WallDuration end(const std::string msg = "")
  {
    const ros::WallDuration dt{ checkpoint(msg) };
    _ofs << "\n";
    _started = false;

    return dt;
  }

protected:
  ros::WallTime _start;
  bool _started;

  std::ofstream _ofs;
};
}  // namespace utils
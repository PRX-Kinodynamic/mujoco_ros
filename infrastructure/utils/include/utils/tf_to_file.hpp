#include <unordered_set>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>

namespace utils
{
template <class Base>
class tf_to_file_t : public Base
{
  using Derived = tf_to_file_t<Base>;

public:
  tf_to_file_t() : _tf_listener(_tf_buffer)
  {
  }

  ~tf_to_file_t()
  {
    _ofs.close();
  }

  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    // ros::NodeHandle private_nh("~");

    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };
    std::string filename;

    PARAM_SETUP(private_nh, filename);
    PARAM_SETUP(private_nh, world_frame);
    PARAM_SETUP(private_nh, robot_frame);

    _ofs.open(filename, std::ios::trunc);

    _ofs << "# dt x y z qw qx qy qz \n";

    const ros::Duration freq_timer(1.0 / 30.0);
    _timer = private_nh.createTimer(freq_timer, &Derived::timer_callback, this);
  }

protected:
  void timer_callback(const ros::TimerEvent& event)
  {
    if (update_tf())
    {
      //  dt        x       y         z         qw          qx        qy        qz
      // 0.100178 1.0006 0.0154945 0.0315752 0.708138 -0.000827854 0.00057327 0.706073
      const double dt{ (event.current_real - event.last_real).toSec() };
      _ofs << dt << " ";
      _ofs << _tf.transform.translation.x << " ";
      _ofs << _tf.transform.translation.y << " ";
      _ofs << _tf.transform.translation.z << " ";

      _ofs << _tf.transform.rotation.w << " ";
      _ofs << _tf.transform.rotation.x << " ";
      _ofs << _tf.transform.rotation.y << " ";
      _ofs << _tf.transform.rotation.z << " ";
      _ofs << "\n";
    }
  }
  bool update_tf()
  {
    try
    {
      _tf = _tf_buffer.lookupTransform(_world_frame, _robot_frame, ros::Time(0));
      return true;
    }
    catch (tf2::TransformException& ex)
    {
    }
    return false;
  }

  std::ofstream _ofs;

  ros::Timer _timer;

  // TF
  std::string _world_frame;
  std::string _robot_frame;
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  geometry_msgs::TransformStamped _tf;
};
}  // namespace utils
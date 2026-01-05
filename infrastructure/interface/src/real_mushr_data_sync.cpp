#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <ros/subscriber.h>
#include <tf2_ros/transform_listener.h>

#include <utils/dbg_utils.hpp>
#include <utils/std_utils.hpp>
#include <utils/execution_status.hpp>
#include <utils/rosparams_utils.hpp>
#include <ml4kp_bridge/defs.h>
#include <prx_models/mushr.hpp>

struct collector_t
{
  using This = collector_t;
  bool _collecting;

  std::ofstream _tf_ofs, _ctrl_ofs;

  ros::Timer _timer;
  ros::Subscriber _ackermann_sub;

  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;

  geometry_msgs::TransformStamped _tf;

  std::string _world_frame;
  std::string _robot_frame;

  collector_t(ros::NodeHandle& nh) : _tf_listener(_tf_buffer), _collecting(false)
  {
    const std::string ackermann_topic{ "/mushr/mux/ackermann_cmd_mux/output" };

    std::string output_dir;
    std::string file_id;
    std::string& world_frame{ _world_frame };
    std::string& robot_frame{ _robot_frame };

    PARAM_SETUP(nh, output_dir);
    PARAM_SETUP(nh, file_id);
    PARAM_SETUP(nh, world_frame);
    PARAM_SETUP(nh, robot_frame);

    const std::string tf_file{ output_dir + "/tf_" + file_id + ".txt" };
    const std::string ctrl_file{ output_dir + "/ctrls_" + file_id + ".txt" };

    _tf_ofs.open(tf_file, std::ios::trunc);
    _ctrl_ofs.open(ctrl_file, std::ios::trunc);

    _tf_ofs << "# dt x y z qw qx qy qz \n";
    _ctrl_ofs << "# steering_angle speed acceleration stamp\n";

    const ros::Duration freq_timer(1.0 / 30.0);

    _ackermann_sub = nh.subscribe(ackermann_topic, 1, &This::ackermann_callback, this);

    _timer = nh.createTimer(freq_timer, &This::tf_timer_callback, this);
  }

  ~collector_t()
  {
    _tf_ofs.close();
    _ctrl_ofs.close();
  }

  void ackermann_callback(const ackermann_msgs::AckermannDriveStampedConstPtr msg)
  {
    _collecting = msg->drive.acceleration != 0.0 or msg->drive.speed != 0.0;
    if (_collecting)
    {
      const std::string dt{ prx::utilities::convert_to<std::string>(msg->header.stamp.toSec()) };
      _ctrl_ofs << msg->drive.steering_angle << " ";
      _ctrl_ofs << msg->drive.speed << " ";
      _ctrl_ofs << msg->drive.acceleration << " ";
      _ctrl_ofs << dt << "\n";
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

  void tf_timer_callback(const ros::TimerEvent& event)
  {
    if (_collecting and update_tf())
    {
      //  dt        x       y         z         qw          qx        qy        qz
      // 0.100178 1.0006 0.0154945 0.0315752 0.708138 -0.000827854 0.00057327 0.706073
      // const double dt{ (event.current_real - event.last_real).toSec() };
      const std::string dt{ prx::utilities::convert_to<std::string>(event.current_real.toSec()) };
      _tf_ofs << dt << " ";
      _tf_ofs << _tf.transform.translation.x << " ";
      _tf_ofs << _tf.transform.translation.y << " ";
      _tf_ofs << _tf.transform.translation.z << " ";

      _tf_ofs << _tf.transform.rotation.w << " ";
      _tf_ofs << _tf.transform.rotation.x << " ";
      _tf_ofs << _tf.transform.rotation.y << " ";
      _tf_ofs << _tf.transform.rotation.z << " ";
      _tf_ofs << "\n";
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RealMushrDataSync");
  ros::NodeHandle nh("~");

  collector_t collector(nh);

  ros::spin();

  return 0;
}
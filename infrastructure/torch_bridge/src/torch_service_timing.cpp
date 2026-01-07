#include <numeric>

#include <ros/ros.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <torch_bridge/query_utils.hpp>
// #include <torch_bridge/TorchQuery.h>

struct torch_profiler_t
{
  ros::ServiceClient _torch_service_client;

  torch_bridge::TorchQuery _torch_service_call;

  Eigen::Vector2d _ui;
  Eigen::Vector3d _xi;
  Eigen::Vector3d _x_res;
  Eigen::Matrix<double, 3, 3> _dres_dx;
  Eigen::Matrix<double, 3, 2> _dres_du;

  bool _verbose;
  int _total_calls;

  torch_profiler_t(ros::NodeHandle& nh) : _verbose(false), _total_calls(10)
  {
    nh.getParam("verbose", _verbose);
    nh.getParam("total_calls", _total_calls);

    _torch_service_client = nh.serviceClient<torch_bridge::TorchQuery>("/torch/service", true);
    _torch_service_call.request.inputs = 2;
    _torch_service_call.request.input_dimensions = { 3, 2 };
  }
  void run(const bool with_jacobians)
  {
    std::vector<double> durations;
    for (int i = 0; i < _total_calls; ++i)
    {
      const ros::Time start(ros::Time::now());
      call_service(with_jacobians);
      const ros::Time end(ros::Time::now());
      durations.push_back((end - start).toSec());
    }
    const auto [min, max] = std::minmax_element(begin(durations), end(durations));

    const double sum_durations{ std::reduce(durations.begin(), durations.end()) };  // Get the sum
    const double mean{ sum_durations / _total_calls };

    std::cout << (with_jacobians ? "With Jacobians" : "Without Jacobians") << "\n";
    std::cout << "\tMin: " << *min << "\n";
    std::cout << "\tMax: " << *max << "\n";
    std::cout << "\tMean: " << mean << "\n";
  }
  void call_service(const bool with_jacobians)
  {
    _torch_service_call.request.compute_jacobians = with_jacobians;
    if (_torch_service_client.exists())
    {
      ROS_INFO_ONCE("Starting torch service timing");

      _torch_service_call.request.data.clear();

      _xi = Eigen::Vector3d::Random();
      _ui = Eigen::Vector2d::Random();
      torch_bridge::update_request(_torch_service_call, _xi, _ui);
      if (_torch_service_client.call(_torch_service_call))
      {
        if (_verbose)
        {
          std::cout << "Response:\n" << _torch_service_call.response << "\n";
        }
        torch_bridge::get_result(_torch_service_call, _x_res);
        if (with_jacobians)
        {
          torch_bridge::get_jacobian(_torch_service_call, _dres_dx, _dres_du);
        }
        if (_verbose)
        {
          std::cout << "Xi: " << _xi.transpose() << "\n";
          std::cout << "Ui: " << _ui.transpose() << "\n";
          std::cout << "Xres: " << _x_res.transpose() << "\n";
          std::cout << "Hx:\n" << _dres_dx << "\n";
          std::cout << "Hu:\n" << _dres_du << "\n";
        }
      }
      else
      {
        ROS_ERROR("[Torch Service Timing]: Service call failed!");
      }
    }
    else
    {
      ROS_ERROR("[Torch Service Timing]: Service does not exists!");
    }
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "TorchServiceTimming" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  torch_profiler_t profiler(nh);

  profiler.run(false);
  profiler.run(true);
}

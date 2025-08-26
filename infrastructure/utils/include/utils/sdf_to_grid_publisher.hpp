#include <iostream>

#include <grid_map_msgs/GridMap.h>

// #include <motion_planning/sdf_factor.hpp>
#include <utils/signed_distance_field.hpp>

namespace utils
{
template <class Base>
class sdf_to_grid_publisher_t : public Base
{
  using Derived = sdf_to_grid_publisher_t<Base>;

  using Sdf = utils::signed_distance_field_t;
  using SdfPtr = std::shared_ptr<Sdf>;

public:
  sdf_to_grid_publisher_t()
  {
  }

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };
    std::string sdf_params;
    std::string environment;
    std::string grid_topicname;

    PARAM_SETUP(private_nh, sdf_params);
    // PARAM_SETUP(private_nh, environment)
    PARAM_SETUP(private_nh, grid_topicname);
    // PARAM_SETUP_WITH_DEFAULT(private_nh, sdf_params, sdf_params)

    prx::param_loader sdf_param_loader{};
    sdf_param_loader = Sdf::default_parameters();
    sdf_param_loader.add_file(sdf_params);

    // sdf_param_loader["environment"].set(environment);
    _sdf = Sdf::create(sdf_param_loader);
    _sdf->to_file();

    _grid_publisher = private_nh.advertise<grid_map_msgs::GridMap>(grid_topicname, 1, true);

    populate_grid_msg();

    _grid_publisher.publish(_grid);
  }

  void populate_grid_msg()
  {
    _grid.info.header.stamp = ros::Time::now();
    _grid.info.header.frame_id = "world";

    // # Pose of the grid map center in the frame defined in `header` [m].
    _grid.info.pose.position.x = 0;
    _grid.info.pose.position.y = 0;
    _grid.info.pose.position.z = 0;

    _grid.info.pose.orientation.w = 1;
    _grid.info.pose.orientation.x = 0;
    _grid.info.pose.orientation.y = 0;
    _grid.info.pose.orientation.z = 0;

    _grid.layers.push_back("distance");
    _grid.data.emplace_back();  // distance layer

    const Eigen::Vector2d min_bound{ _sdf->min_bound() };
    const Eigen::Vector2d max_bound{ _sdf->max_bound() };
    const double resolution{ _sdf->resolution() };

    const int rows{ _sdf->rows() };
    const int cols{ _sdf->cols() };
    //     MultiArrayDimension[] dim # Array of dimension properties
    // uint32 data_offset        # padding elements at front of data

    _grid.data[0].layout.data_offset = 0;
    _grid.data[0].layout.dim.emplace_back();
    _grid.data[0].layout.dim.emplace_back();
    _grid.data[0].layout.dim[0].label = "x";
    _grid.data[0].layout.dim[0].size = rows;
    _grid.data[0].layout.dim[0].stride = cols * rows;

    _grid.data[0].layout.dim[1].label = "y";
    _grid.data[0].layout.dim[1].size = cols;
    _grid.data[0].layout.dim[1].stride = cols;

    _grid.data[0].data.resize(cols * rows, 0.0);

    int i{ 0 };
    int j{ 0 };
    DEBUG_VARS(min_bound.transpose());
    DEBUG_VARS(max_bound.transpose());
    for (double xi{ min_bound[0] }; xi < max_bound[0]; xi += resolution)
    {
      j = 0;
      // DEBUG_VARS(i, j);
      for (double yi{ min_bound[1] }; yi < max_bound[1]; yi += resolution)
      {
        const double distance(_sdf->distance(xi, yi));
        // DEBUG_VARS(xi, yi, distance);
        add_to_grid(0, i, j, distance);
        j++;
      }
      i++;
    }

    // # Resolution of the grid [m/cell].
    _grid.info.resolution = resolution;

    // // # Length in x-direction [m].
    _grid.info.length_x = max_bound[0] - min_bound[0];

    // // # Length in y-direction [m].
    _grid.info.length_y = max_bound[1] - min_bound[1];

    // # Pose of the grid map center in the frame defined in `header` [m].
    _grid.info.pose.position.x = (max_bound[0] + min_bound[0]) / 2.0;
    _grid.info.pose.position.y = (max_bound[1] + min_bound[1]) / 2.0;

    _grid.info.pose.orientation.w = 0.0;
    _grid.info.pose.orientation.x = 0.0;
    _grid.info.pose.orientation.y = 0.0;
    _grid.info.pose.orientation.z = 1.0;
  }

  void add_to_grid(const int layer, const int i, const int j, const double value)
  {
    const unsigned int& data_offset{ _grid.data[layer].layout.data_offset };
    const unsigned int& stride_1{ _grid.data[layer].layout.dim[1].stride };
    // const std::size_t idx{ data_offset + stride_1 * i + j };
    const std::size_t idx{ data_offset + stride_1 * j + i };
    // DEBUG_VARS(idx);
    // multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    _grid.data[layer].data[idx] = value;
  }

  // sensor_msgs::ImagePtr _msg, _msg_rgb;

  // ros::Subscriber _image_subscriber;
  ros::Publisher _grid_publisher;

  SdfPtr _sdf;
  grid_map_msgs::GridMap _grid;
};

}  // namespace utils

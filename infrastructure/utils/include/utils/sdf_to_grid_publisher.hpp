#include <iostream>

#include <grid_map_msgs/GridMap.h>
#include <std_msgs/Float32MultiArray.h>

// #include <motion_planning/sdf_factor.hpp>
#include <utils/signed_distance_field.hpp>
#include "utils/dbg_utils.hpp"
#include "utils/rosparams_utils.hpp"
// #include <interface/node_status.hpp>

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

    // _node_status = interface::node_status_t::create(nh, "/nodes/grid/");

    // PARAM_SETUP(private_nh, sdf_params);
    // PARAM_SETUP(private_nh, environment)
    PARAM_SETUP(private_nh, grid_topicname);
    // PARAM_SETUP_WITH_DEFAULT(private_nh, sdf_params, sdf_params)

    // prx::param_loader sdf_param_loader{};
    // sdf_param_loader = Sdf::default_parameters();
    // sdf_param_loader.add_file(sdf_params);

    // sdf_param_loader["environment"].set(environment);
    _sdf = Sdf::create(private_nh);
    _sdf->to_file();

    _grid_publisher = private_nh.advertise<grid_map_msgs::GridMap>(grid_topicname, 1, true);

    populate_grid_msg();

    _grid_publisher.publish(_grid);
    // node_status->status(interface::NodeStatus::RUNNING);
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
    // _grid.layers.push_back("distance2");
    // _grid.basic_layers.push_back("distance");
    _grid.data.emplace_back();  // distance layer
    // _grid.data.emplace_back();  // distance layer

    const Eigen::Vector2d min_bound{ _sdf->min_bound() };
    const Eigen::Vector2d max_bound{ _sdf->max_bound() };
    const double resolution{ _sdf->resolution() };

    const int rows{ _sdf->rows() };
    const int cols{ _sdf->cols() };
    //     MultiArrayDimension[] dim # Array of dimension properties
    // uint32 data_offset        # padding elements at front of data
    const int data_offset{ 0 };

    // std_msgs::MultiArrayLayout& layout{ _grid.data[0].layout };

    matrixEigenCopyToMultiArrayMessage(_sdf->sdf_matrix(), _grid.data[0]);
    // matrixEigenCopyToMultiArrayMessage(_sdf->sdf_matrix(), _grid.data[1]);

    // std_msgs::MultiArrayLayout& layout{ _grid.data[0].layout };
    // layout.data_offset = data_offset;
    // layout.dim.emplace_back();
    // layout.dim.emplace_back();

    // layout.dim[0].label = "y";
    // layout.dim[0].size = rows;
    // layout.dim[0].stride = 10;

    // layout.dim[1].label = "x";
    // layout.dim[1].size = cols;
    // layout.dim[1].stride = 10;

    // // DEBUG_VARS(rows, cols)
    // _grid.data[0].data.resize(cols * rows, 0.0);

    // // *  y = i / row_stride * cell_size.y
    // // *  x = (i % row_stride) / cell_stride * cell_size.x

    // int i{ 0 };
    // int j{ 0 };
    // // int idx{ 0 };
    // // DEBUG_VARS(min_bound.transpose());
    // // DEBUG_VARS(max_bound.transpose());
    // // for (double xi{ min_bound[0] }; xi < max_bound[0]; xi += resolution)
    // for (double xi{ max_bound[0] }; min_bound[0] < xi; xi -= resolution)
    // {
    //   j = 0;
    //   // DEBUG_VARS(i, j);
    //   // for (double yi{ min_bound[1] }; yi < max_bound[1]; yi += resolution)
    //   for (double yi{ max_bound[1] }; min_bound[1] < yi; yi -= resolution)
    //   {
    //     const double distance(_sdf->distance(xi, yi));
    //     // DEBUG_VARS(xi, yi, distance);
    //     // add_to_grid(0, i, j, distance);
    //     // _grid.data[0].data[idx] = distance;
    //     _grid.data[0].data[j * cols + i] = distance;
    //     // const std::size_t idx{ data_offset + layout.dim[1].stride * i + j };
    //     // DEBUG_VARS(layout.dim[1].stride, i, j, idx)
    //     // _grid.data[0].data[idx] = distance;
    //     // # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]

    //     // idx++;
    //     j++;
    //     // DEBUG_PRINT
    //   }
    //   i++;
    // }

    // # Resolution of the grid [m/cell].
    _grid.info.resolution = resolution;

    // // # Length in x-direction [m].
    _grid.info.length_x = 7.1;  // max_bound[0] - min_bound[0];

    // // # Length in y-direction [m].
    _grid.info.length_y = 2.7;  // max_bound[1] - min_bound[1];

    // # Pose of the grid map center in the frame defined in `header` [m].
    _grid.info.pose.position.x = (max_bound[0] + min_bound[0]) / 2.0;
    _grid.info.pose.position.y = (max_bound[1] + min_bound[1]) / 2.0;

    _grid.info.pose.orientation.w = 1.0;
    _grid.info.pose.orientation.x = 0.0;
    _grid.info.pose.orientation.y = 0.0;
    _grid.info.pose.orientation.z = 0.0;
  }

  template <typename EigenType_, typename MultiArrayMessageType_>
  bool matrixEigenCopyToMultiArrayMessage(const EigenType_& e, MultiArrayMessageType_& m)
  {
    m.layout.dim.resize(2);
    m.layout.dim[0].stride = e.size();
    m.layout.dim[0].size = e.outerSize();
    m.layout.dim[1].stride = e.innerSize();
    m.layout.dim[1].size = e.innerSize();
    // m.layout.dim[2].stride = 1;
    // m.layout.dim[2].size = 1;

    if (e.IsRowMajor)
    {
      m.layout.dim[0].label = "Row";
      m.layout.dim[1].label = "Column";
    }
    else
    {
      m.layout.dim[0].label = "Column";
      m.layout.dim[1].label = "Row";
    }

    m.data.clear();
    m.data.insert(m.data.begin(), e.data(), e.data() + e.size());
    return true;
  }

  // sensor_msgs::ImagePtr _msg, _msg_rgb;

  // ros::Subscriber _image_subscriber;
  ros::Publisher _grid_publisher;

  SdfPtr _sdf;
  grid_map_msgs::GridMap _grid;

  // std::shared_ptr<interface::node_status_t> _node_status;
};

}  // namespace utils
#include <memory>
#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>
#include <utils/dbg_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/SO2_system.hpp>

#include <motion_planning/stela_sliding_window.hpp>
#include <motion_planning/tree_validation.hpp>

#include <interface/node_status.hpp>
#include <interface/ExperimentParams.h>
#include <interface/levenberg_marquardt_interface.hpp>
#include <interface/StelaStatus.h>
#include <std_msgs/Int32.h>
#include <control/mushr_contingency_controllers.hpp>
#include <motion_planning/goal_checker.hpp>
#include <motion_planning/safety_checker.hpp>
#include <motion_planning/randup.hpp>
#include <prx_models/mushr.hpp>

// #include <prx_models/mushr_torch.hpp>
#include <prx_models/mushr_mujoco.hpp>
#include <prx_models/StelaKraft.h>
#include <motion_planning/morse_graph_reachability.hpp>
#include <prx/utilities/data_structures/implicit_grid.hpp>
#include <ml4kp_bridge/defs.h>
#include <prx/utilities/general/type_conversions.hpp>

using prx::utilities::convert_to;

template <typename State>
struct gt_cell_t
{
  gt_cell_t(const State& state_, bool safe_, int total_states_)
    : state(state_), safe(safe_), total_states(total_states_)
  {
  }

  bool safe;
  State state;
  int total_states;
};

template <typename DynamicalSystem, typename Controller>
struct gt_merger_t
{
  using State = typename DynamicalSystem::State;
  using Cell = gt_cell_t<State>;
  using CellPtr = std::shared_ptr<Cell>;
  using ImplicitGrid = prx::implicit_grid_t<State, CellPtr>;
  using Line = std::vector<std::string>;

  ImplicitGrid _grid;

  Line line;

  int id0, id1;
  gt_merger_t(ros::NodeHandle& nh)
  {
    std::string grid_filename_0, grid_filename_1;
    PARAM_SETUP(nh, grid_filename_0);
    PARAM_SETUP(nh, grid_filename_1);

    prx::utilities::csv_reader_t reader0(grid_filename_0);
    prx::utilities::csv_reader_t reader1(grid_filename_1);

    reader0.next_valid_line(line);

    // ID  X0(2) cellsize(2)
    // 100  0 0  0.001 0.001
    id0 = convert_to<double>(line[0]);
    const double x0_0{ convert_to<double>(line[1]) };
    const double x0_1{ convert_to<double>(line[2]) };
    const double c0{ convert_to<double>(line[3]) };
    const double c1{ convert_to<double>(line[4]) };

    const State x0(gtsam::Rot2(x0_0), x0_1);
    typename ImplicitGrid::TangentElement cellsize(c0, c1);
    _grid.reset(x0, cellsize);

    reader1.next_valid_line(line);
    id1 = convert_to<double>(line[0]);
    prx_assert(id0 == id1, "[gt_grid_merge] Ids do not match");

    fill_grid(reader0);
    fill_grid(reader1);
    to_file(grid_filename_0);
  }

  void fill_grid(prx::utilities::csv_reader_t& reader)
  {
    // X0-0     X0-1  Safe
    // -2.7025 5.7385 1
    while (reader.next_valid_line(line))
    {
      const double x0_0{ convert_to<double>(line[0]) };
      const double x0_1{ convert_to<double>(line[1]) };
      const int safe{ convert_to<int>(line[2]) };
      const int total_states{ convert_to<int>(line[3]) };

      const State x0(gtsam::Rot2(x0_0), x0_1);
      if (not _grid.exists(x0))
      {
        _grid.cell(x0) = std::make_shared<Cell>(x0, safe, total_states);
      }
      else
      {
        _grid.cell(x0)->total_states += total_states;
      }
    }
  }

  void to_file(const std::string outputfile)
  {
    std::ofstream ofs(outputfile.c_str());
    ofs << "# First line: 'id x0 cell_size' of grid (the id of this reachable set, x0 is the x0 of the grid  ";
    ofs << "and the size of each cell). Then empty line and then N lines with 'states safe' ";
    ofs << "corresponding to the reachable set (on the grid).\n";
    prx::to_stream(ofs, id0);
    prx::to_stream(ofs, _grid.x0());
    prx::to_stream(ofs, _grid.cell_sizes());
    ofs << "\n\n";

    for (auto cell : _grid)
    {
      prx::to_stream(ofs, cell.second->state);
      prx::to_stream(ofs, cell.second->safe);
      prx::to_stream(ofs, cell.second->total_states);
      ofs << "\n";
    }
    ofs.close();
    DEBUG_VARS(outputfile);
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "MushrSbmpOpenLoop" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  using SO2PieceWiseStep = prx::piecewise_step_t<prx::SO2_system_t::Control, double>;
  using SO2Controller = std::vector<SO2PieceWiseStep>;

  using MushrPieceWiseStep = prx::piecewise_step_t<prx::mushrPolynomial_t::Control, double>;
  using MushrController = std::vector<MushrPieceWiseStep>;

  using SO2HelperPiecewise = gt_merger_t<prx::SO2_system_t, SO2Controller>;
  using MushrHelperPiecewise = gt_merger_t<prx::mushrPolynomial_t, MushrController>;

  std::shared_ptr<SO2HelperPiecewise> SO2_helper;
  std::shared_ptr<MushrHelperPiecewise> mushr_helper;

  std::string plant;
  // PARAM_SETUP(nh, plant)

  // if (plant == "SO2System")
  // {
  SO2_helper = std::make_shared<SO2HelperPiecewise>(nh);
  // }
  // else if (plant == "mushrPolynomial")
  // {
  //   mushr_helper = std::make_shared<MushrHelperPiecewise>(nh);
  // }
  // else { prx_throw("Invalid 'plant' parameter") }

  // ros::spin();

  return 0;
}
#include <thread>

#include <ml4kp_bridge/defs.h>
#include <utils/std_utils.hpp>

#include <prx_models/mushr.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <prx_models/mushr_torch.hpp>
#include <prx_models/mushr_mujoco.hpp>
#include <motion_planning/stela_sliding_window.hpp>
#include <utils/dbg_utils.hpp>
#include <interface/node_status.hpp>
#include <interface/ExperimentParams.h>
#include <prx/planning/planners/aorrt.hpp>
#include <prx_models/defs.hpp>

struct planning_experiments_t
{
  std::ofstream _ofs;  //(output_filename);

  std::shared_ptr<prx::world_model_t> _planning_model;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;

  ros::Publisher _tree_publisher;
  ros::Publisher _sln_tree_publisher;
  std::string planner_file;

  planning_experiments_t(ros::NodeHandle& nh)
  {
    std::string plant_file, environment_file, output_filename;
    int random_seed;
    using prx::simulation_step;
    PARAM_SETUP(nh, random_seed);
    PARAM_SETUP(nh, simulation_step);

    PARAM_SETUP(nh, plant_file);
    PARAM_SETUP(nh, planner_file);
    PARAM_SETUP(nh, environment_file);

    // PARAM_SETUP(nh, tree_topic);
    PARAM_SETUP(nh, output_filename);

    _ofs.open(output_filename);

    prx::init_random(random_seed);

    prx::param_loader plant_params(plant_file);
    prx::param_loader environment_params(environment_file);

    prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_params) };
    std::tie(_planning_model, _system_group, _collision_group) = prx::world_model_t::create(environment_params, plant);

    _tree_publisher = nh.advertise<prx_models::Tree>("/aorrt/tree", 1, true);
    _sln_tree_publisher = nh.advertise<prx_models::Tree>("/aorrt/sln_tree", 1, true);
  }

  ~planning_experiments_t()
  {
    _ofs.close();
  }

  double traverse_tree_sln(std::shared_ptr<prx::aorrt_t::Node> node, std::shared_ptr<prx::tree_t> tree_slns)
  {
    auto children = node->get_children();
    if (children.size() == 0)
    {
      _ofs << "\n";
      return node->cost();  // the last element is the cost
    }
    double cost{ std::numeric_limits<double>::max() };
    while (children.size() > 0)
    {
      auto next_node = tree_slns->get_vertex_as<prx::aorrt_t::Node>(children.front());
      const double child_cost{ traverse_tree_sln(next_node, tree_slns) };
      cost = std::min(child_cost, cost);
      children.pop_front();
    }
    const double cost_to_go{ cost - node->cost() };
    _ofs << node->point << " ";
    _ofs << cost_to_go << " ";
    _ofs << "\n";

    return cost;
  }

  void sample_x0(prx::space_point_t x0)
  {
    do
    {
      _system_group->get_state_space()->sample(x0);
      _system_group->get_state_space()->copy_from(x0);
    } while (_collision_group->in_collision());
  }

  void run_planner()
  {
    prx::param_loader planner_params(planner_file);
    prx::aorrt_t aorrt("aorrt");
    prx::aorrt_specification_t aorrt_spec(_system_group, _collision_group);
    prx::aorrt_query_t aorrt_query(_system_group->get_state_space(), _system_group->get_control_space());
    prx::condition_check_t checker(planner_params["checker"]);

    aorrt_spec.init(planner_params["specification"]);
    aorrt_query.init(planner_params["query"]);

    const prx_models::mushr_types::State::type goal{ Vec(aorrt_query.goal_state) };

    sample_x0(aorrt_query.start_state);
    DEBUG_VARS(aorrt_query.start_state, goal)

    aorrt_query.goal_check = [&, goal](prx::space_point_t s) {
      prx_models::mushr_types::State::type xi(Vec(s));
      const prx_models::mushr_types::State::type between{ xi.between(goal) };
      const Eigen::Vector3d error{ prx_models::mushr_types::State::type::Logmap(between) };
      const double goal_error{ error.norm() };
      return goal_error < aorrt_query.goal_region_radius;
    };

    aorrt.link_and_setup_spec(&aorrt_spec);
    aorrt.preprocess();
    aorrt.link_and_setup_query(&aorrt_query);

    aorrt.resolve_query(&checker);
    aorrt.fulfill_query();

    std::shared_ptr<prx::tree_t> solutions{ aorrt.tree_of_solutions() };

    // PRINT_MSG("Planning finished")
    // delete vis_group;
    prx_models::Tree full_tree, sln_tree;
    motion_planning::copy<prx::aorrt_t::Node, prx::aorrt_t::Edge>(full_tree, aorrt.tree());
    motion_planning::copy<prx::aorrt_t::Node, prx::aorrt_t::Edge>(sln_tree, *solutions);
    _tree_publisher.publish(full_tree);
    _sln_tree_publisher.publish(sln_tree);
    // PRINT_MSG("Trees published!")

    auto root_idx = aorrt.root_index();
    std::shared_ptr<prx::aorrt_t::Node> root{ solutions->get_vertex_as<prx::aorrt_t::Node>(root_idx) };

    traverse_tree_sln(root, solutions);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RosAORRT");
  ros::NodeHandle nh("~");

  planning_experiments_t experiments(nh);

  for (int i = 0; i < 100; ++i)
  {
    experiments.run_planner();
  }

  PRINT_MSG("Experiments finished!");
  ros::spin();

  return 0;
}
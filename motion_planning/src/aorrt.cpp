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

// Mujoco-Ros visualization in (almost) RT:
// Depends on the vizualization thread, but if the viz thread slows down, it won't affect mujoco
int main(int argc, char** argv)
{
  ros::init(argc, argv, "RosAORRT");
  ros::NodeHandle nh("~");

  int random_seed;
  using prx::simulation_step;
  std::string plant_file, environment_file, planner_file, tree_topic;

  PARAM_SETUP(nh, random_seed);
  PARAM_SETUP(nh, simulation_step);

  PARAM_SETUP(nh, plant_file);
  PARAM_SETUP(nh, planner_file);
  PARAM_SETUP(nh, environment_file);

  PARAM_SETUP(nh, tree_topic);

  ros::Publisher tree_publisher{ nh.advertise<prx_models::Tree>("/aorrt/tree", 1, true) };

  prx::init_random(random_seed);
  prx::param_loader plant_params(plant_file);
  prx::param_loader planner_params(planner_file);
  prx::param_loader environment_params(environment_file);

  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_params) };
  auto [planning_model, system_group, collision_group] = prx::world_model_t::create(environment_params, plant);

  prx::aorrt_t aorrt("aorrt");
  prx::aorrt_specification_t aorrt_spec(system_group, collision_group);
  prx::aorrt_query_t aorrt_query(system_group->get_state_space(), system_group->get_control_space());
  prx::condition_check_t checker(planner_params["checker"]);

  aorrt_spec.init(planner_params["specification"]);
  aorrt_query.init(planner_params["query"]);

  const prx_models::mushr_types::State::type goal{ Vec(aorrt_query.goal_state) };
  DEBUG_VARS(goal)
  aorrt_query.goal_check = [&, goal](prx::space_point_t s) {
    prx_models::mushr_types::State::type xi(Vec(s));
    // xi[0] = s->at(0);
    // xi[1] = s->at(1);
    // xi[2] = s->at(2);
    // xg[0] = _dirt_query->goal_state->at(0);
    // xg[1] = _dirt_query->goal_state->at(1);
    // xg[2] = _dirt_query->goal_state->at(2);
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

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { planning_model->get_obstacles() });

  const std::string body_name{ plant_params["name"].as<>() + "/" + plant_params["vis_body"].as<>() };

  vis_group->add_vis_infos(prx::info_geometry_t::LINE, aorrt_query.tree_visualization, body_name,
                           system_group->get_state_space());
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, aorrt_query.solution_traj, body_name,
                                    system_group->get_state_space());
  vis_group->add_animation(aorrt_query.solution_traj, system_group->get_state_space(), aorrt_query.start_state);
  vis_group->output_html("aorrt_output.html");

  delete vis_group;
  prx_models::Tree ros_tree;
  motion_planning::copy<prx::aorrt_t::Node, prx::aorrt_t::Edge>(ros_tree, aorrt.tree());
  tree_publisher.publish(ros_tree);

  ros::spin();
  //////////////////////////
  // auto params = param_loader("examples/basic/aorrt.yaml", argc, argv);

  // simulation_step = params["simulation_step"].as<double>();
  // init_random(params["random_seed"].as<int>());

  // auto obstacles = load_obstacles(params["environment"].as<>());
  // std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
  // std::vector<std::string> obstacle_names = obstacles.first;

  // std::string plant_name = params["/plant/name"].as<>();
  // std::string plant_path = params["/plant/path"].as<>();
  // auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  // prx_assert(plant != nullptr, "Plant is nullptr!");
  // plant->init(params["plant"]);

  // world_model_t world_model({ plant }, { obstacle_list });
  // world_model.create_context("context", { plant_name }, { obstacle_names });
  // auto context = world_model.get_context("context");

  // aorrt_t aorrt(params["planner"].as<>());
  // aorrt_specification_t aorrt_spec(context.first, context.second);

  // rrt_spec.valid_state = [](space_point_t& s)
  // {
  // Custom valid_state can be added here.
  // };

  // rrt_spec.valid_check = [&rrt_spec](trajectory_t& traj)
  // {
  // Custom valid_check goes here...
  // Basically for x in traj, call valid_state
  // };

  // Two ways of accessing lengthy parameter paths
  // int min_steps = params["plant"]["steps"]["min"].as<int>();
  // int max_steps = params["/plant/steps/max"].as<int>();

  // aorrt_spec.min_control_steps = min_steps;
  // aorrt_spec.max_control_steps = max_steps;

  // aorrt_query_t aorrt_query(context.first->get_state_space(), context.first->get_control_space());
  // aorrt_query.start_state = context.first->get_state_space()->make_point();
  // aorrt_query.goal_state = context.first->get_state_space()->make_point();

  // auto lower_bounds = params["/plant/state_space/lower_bound"].as<std::vector<double>>();
  // auto upper_bounds = params["/plant/state_space/upper_bound"].as<std::vector<double>>();
  // context.first->get_state_space()->set_bounds(lower_bounds, upper_bounds);

  // context.first->get_state_space()->copy(aorrt_query.start_state,
  //                                        params["/plant/start_state"].as<std::vector<double>>());
  // context.first->get_state_space()->copy(aorrt_query.goal_state,
  // params["/plant/goal/state"].as<std::vector<double>>());

  // aorrt_query.goal_region_radius = params["/plant/goal/radius"].as<double>();

  // Alternatively, change the goal_check function
  // rrt_query.goal_check = [&](space_point_t pt)
  // {
  //    // Default is:
  // return space_t::euclidean_2d(pt, rrt_query.goal_state) < goal_region_radius;
  // }

  // aorrt_query.get_visualization = params["visualize"].as<bool>();

  // aorrt.link_and_setup_spec(&aorrt_spec);
  // aorrt.preprocess();
  // aorrt.link_and_setup_query(&aorrt_query);

  // condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

  // std::cout << "Running " << params["planner"].as<>() << " for " << params["checker_value"].as<int>() << " ("
  //           << params["checker_type"].as<>() << ")" << std::endl;

  // aorrt.resolve_query(&checker);
  // aorrt.fulfill_query();

  // params.print();

  // prx::planning::discretize_tree(_planner->tree(), *_planner, _params["/planner/max_edge_duration"].as<double>());

  // aorrt.tree().to_file("/Users/Gary/pracsys/catkin_ws/dbg/tree_before.txt");
  // prx::planning::discretize_tree(aorrt.tree(), aorrt, 0.5);
  // aorrt.tree().to_file("/Users/Gary/pracsys/catkin_ws/dbg/tree_discretized.txt");
  // TODO: Add function to visualization to replace tree_to_txt
  // tree_to_txt(dirt_query);

  // //   }
  // //   catch(const prx_assert_t& e)
  // //   {
  // // std::cout<<e.get_message()<<std::endl;
  // //   }
  // std::cout << "End of program" << std::endl;
  return 0;
}
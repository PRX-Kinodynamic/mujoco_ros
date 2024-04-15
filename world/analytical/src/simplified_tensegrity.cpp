#include <thread>
#include <utils/rosparams_utils.hpp>
#include <utils/nodelet_as_node.hpp>

#include <analytical/ltv_sde.hpp>
#include <analytical/simulator.hpp>
#include <analytical/fg_ltv_sde.hpp>

int main(int argc, char** argv)
{
  DEBUG_PRINT;
  ros::init(argc, argv, "simplified_tensegrity");
  ros::NodeHandle nh("~");
  DEBUG_PRINT;

  std::string prx_config_file;
  PARAM_SETUP(nh, prx_config_file);

  DEBUG_VARS(prx_config_file);

  prx::param_loader params;
  params.set_input_path("/");
  params.add_file(prx_config_file);

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<double>());

  auto obstacles = prx::load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  std::vector<std::string> obstacle_names{ obstacles.first };

  const std::string plant_name{ params["/plant/name"].as<>() };
  const std::string plant_path{ params["/plant/path"].as<>() };

  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_path) };
  prx_assert(plant != nullptr, "Plant is nullptr!");

  plant->init(params["plant"]);

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sg{ context.first };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  prx::space_point_t start_state{ ss->make_point() };

  ss->copy(start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy_from(start_state);

  prx::plan_t plan(cs);
  prx::trajectory_t solution_traj(ss);

  plan.copy_onto_back(Eigen::Vector2d(0.0, 1.0), 10);

  sg->propagate(start_state, plan, solution_traj);

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, solution_traj, body_name, ss);

  vis_group->add_animation(solution_traj, ss, start_state);

  vis_group->output_html("simplified_tensegrity.html");

  delete vis_group;

  return 0;
}
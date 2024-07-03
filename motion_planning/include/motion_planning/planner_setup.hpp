#pragma once
#include <ml4kp_bridge/defs.h>

#include <prx/utilities/defs.hpp>
#include <prx/utilities/general/param_loader.hpp>

#include <prx/simulation/plants/plants.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>

#include <prx/planning/world_model.hpp>
#include <prx/planning/planners/dirt.hpp>
#include <prx/planning/planners/aorrt.hpp>
#include <prx/planning/planners/planner.hpp>

#include <prx/visualization/three_js_group.hpp>

#include <prx_models/Graph.h>
#include <prx_models/NodeEdge.h>
#include <prx_models/Tree.h>

namespace motion_planning
{
struct Config2dFromState
{
  using State = Eigen::Vector2d;
  void operator()(Eigen::Matrix3d& rotation, Eigen::Vector3d& translation, const State& state)
  {
    rotation = Eigen::Matrix3d::Identity();
    translation[0] = state[0];
    translation[1] = state[1];
    translation[2] = 0;
  }

  void operator()(Eigen::MatrixXd& H, const Eigen::Vector3d& translation)
  {
    H = Eigen::Matrix2d::Identity();
    H.diagonal() = translation.head(2);
  }
};

prx::PairNameObstacles setup_world(const prx::param_loader& params, std::shared_ptr<prx::system_t>& plant,
                                   std::shared_ptr<prx::world_model_t>& world_model,
                                   std::shared_ptr<prx::system_group_t>& system_group,
                                   std::shared_ptr<prx::collision_group_t>& collision_group)
{
  const prx::PairNameObstacles obstacles{ prx::load_obstacles(params["planner/environment"].as<>()) };
  const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  const std::vector<std::string> obstacle_names{ obstacles.first };

  const std::string plant_name{ params["/plant/name"].as<>() };
  const std::string plant_path{ params["/plant/path"].as<>() };
  plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");
  plant->init(params["plant"]);

  world_model.reset(new prx::world_model_t({ plant }, { obstacle_list }));
  world_model->create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model->get_context("context");
  system_group = context.first;
  collision_group = context.second;

  DEBUG_VARS("setup_world:", plant);
  // system_group->get_state_space()->init(params["/plant/state_space/"]);
  // system_group->get_control_space()->init(params["/plant/control_space/"]);
  // system_group->get_parameter_space()->init(params["/plant/parameter_space/"]);
  return obstacles;
}

void setup_spec(const prx::param_loader& params, std::shared_ptr<prx::rrt_specification_t>& spec)
{
  spec->init(params["plant"]);
  spec->init(params["planner"]);
}

void setup_spec(const prx::param_loader& params, std::shared_ptr<prx::dirt_specification_t>& spec)
{
  std::shared_ptr<prx::rrt_specification_t> rrt_spec{ std::dynamic_pointer_cast<prx::rrt_specification_t>(spec) };
  setup_spec(params, rrt_spec);
  spec->h = [&](const prx::space_point_t& s1, const prx::space_point_t& s2) {
    return (Vec(s1).head(2) - Vec(s2).head(2)).norm();
  };
}

void setup_spec(const prx::param_loader& params, std::shared_ptr<prx::aorrt_specification_t>& spec)
{
  std::shared_ptr<prx::rrt_specification_t> rrt_spec{ std::dynamic_pointer_cast<prx::rrt_specification_t>(spec) };
  setup_spec(params, rrt_spec);
}

template <typename PlannerQueryPtr>
void setup_query(const prx::param_loader& params, PlannerQueryPtr& query,
                 std::shared_ptr<prx::condition_check_t>& checker)
{
  query->init(params["planner"]);
  query->init(params["plant"]);

  const std::string checker_type{ params["/planner/checker_type"].as<>() };
  const int checker_value{ params["/planner/checker_value"].as<int>() };

  checker = std::make_shared<prx::condition_check_t>(checker_type, checker_value);
}

std::vector<Eigen::Vector2d> setup_medial_axis_sampler(prx::param_loader& params, prx::space_point_t start,
                                                       prx::space_point_t goal)
{
  using State = Eigen::Vector2d;
  using ObstacleFactor = prx::fg::obstacle_factor_t<State, Config2dFromState>;
  using csv_reader_t = prx::utilities::csv_reader_t;
  using Block = csv_reader_t::Block<std::string>;
  using prx::utilities::convert_to;

  Config2dFromState config_from_state;
  ObstacleFactor::CollideResult collide_result;
  const std::string filename{ params["filename"].as<>() };
  const std::string environment{ params["environment"].as<>() };
  const double resolution{ params["resolution"].as<double>() };

  const std::vector<double> robot_params{ params["geometry/parameters"].as<std::vector<double>>() };
  auto obstacles = prx::load_obstacles(environment);
  auto obstacle_collision_infos = prx::fg::collision_info_t::generate_infos(obstacles.second);
  std::shared_ptr<prx::fg::collision_info_t> robot_collision_ptr{ std::make_shared<prx::fg::collision_info_t>(
      prx::geometry_type_t::SPHERE, robot_params) };

  csv_reader_t reader(filename);
  prx::constants::separating_value = ' ';

  std::vector<Eigen::Vector2d> ma;
  std::unordered_map<int, Eigen::Vector2d> ma_nodes{};

  Block block_nodes{ reader.next_block() };
  int max_id{ 0 };
  for (auto line : block_nodes)
  {
    if (line.size() == 0)
      break;
    // Line: id, x, y
    const int id{ convert_to<int>(line[0]) };
    const double x{ convert_to<double>(line[1]) };
    const double y{ convert_to<double>(line[2]) };

    max_id = std::max(id, max_id);
    ma_nodes[id] = State(x, y);
  }

  // Block where each line: id0, id1
  Block block_edges{ reader.next_block() };
  std::vector<std::pair<int, int>> edges;

  for (auto line : block_edges)
  {
    if (line.size() == 0)
      break;
    const int id0{ convert_to<int>(line[0]) };
    const int id1{ convert_to<int>(line[1]) };

    edges.push_back(std::make_pair(id0, id1));
  }

  const int start_id{ max_id + 1 };
  const int goal_id{ max_id + 2 };
  ma_nodes[start_id] = Vec(start).head(2);
  ma_nodes[goal_id] = Vec(goal).head(2);
  for (auto node_pair : ma_nodes)
  {
    const int id{ node_pair.first };

    if (id != start_id)
      edges.push_back(std::make_pair(id, start_id));
    if (id != goal_id)
      edges.push_back(std::make_pair(id, goal_id));
  }

  for (auto pair_nodes : edges)
  {
    const int id0{ pair_nodes.first };
    const int id1{ pair_nodes.second };

    const Eigen::Vector2d p0{ ma_nodes[id0] };
    const Eigen::Vector2d p1{ ma_nodes[id1] };

    std::vector<State> edge;
    bool edge_in_collision = false;
    for (double ti = 0.0; ti <= 1.0; ti += resolution)
    {
      const State pt{ (1.0 - ti) * p0 + ti * p1 };
      edge.push_back(pt);
      for (auto obstacle_info : obstacle_collision_infos)
      {
        edge_in_collision |=
            ObstacleFactor::in_collision(pt, obstacle_info, robot_collision_ptr, config_from_state, collide_result);
        if (edge_in_collision)
        {
          break;
        }
      }
    }
    if (not edge_in_collision)
    {
      ma.insert(ma.end(), edge.begin(), edge.end());
    }
  }

  return ma;
}
}  // namespace motion_planning
#include <utils/dbg_utils.hpp>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <prx_models/defs.hpp>

#include <motion_planning/motion_planning_types.hpp>
#include <motion_planning/tree_bridge.hpp>
#include <motion_planning/planner_setup.hpp>
#include <motion_planning/planner_postprocessing.hpp>

#include <analytical/fg_ltv_sde.hpp>

#include <prx/utilities/defs.hpp>
#include <prx/planning/world_model.hpp>
#include <prx/simulation/plants/plants.hpp>
#include <prx/planning/planners/planner.hpp>
#include <prx/visualization/three_js_group.hpp>
#include <prx/utilities/general/param_loader.hpp>
#include <prx/simulation/loaders/obstacle_loader.hpp>
#include <prx/planning/planner_functions/tree_fix_time_discretization.hpp>

namespace motion_planning
{

template <class Planner, class PlannerSpecification, class PlannerQuery, class Base>
class sbmp_publisher_t : public Base
{
  using Derived = sbmp_publisher_t<Planner, PlannerSpecification, PlannerQuery, Base>;

public:
  sbmp_publisher_t()
    : Base()
    , _tree_topic_name("/tree")
    , _sln_tree_topic_name("/sln_tree")
    , _tree_service_name("/run")
    , _goal_marker_topic_name("/goal")
    , _planner_name("/sbmp")
    , _params()
  {
  }

protected:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh{ Base::getPrivateNodeHandle() };

    std::string planner_ml4kp_params;
    std::string plant_ml4kp_params;
    std::string stela_params{ "" };
    std::string planner_name;
    std::string environment;

    PARAM_SETUP(private_nh, planner_ml4kp_params);
    PARAM_SETUP(private_nh, plant_ml4kp_params);
    PARAM_SETUP(private_nh, environment);
    PARAM_SETUP_WITH_DEFAULT(private_nh, planner_name, _planner_name);
    PARAM_SETUP_WITH_DEFAULT(private_nh, stela_params, stela_params);

    _params["plant"] = prx::param_loader(plant_ml4kp_params, "");
    _params["planner"] = prx::param_loader(planner_ml4kp_params, "");
    if (stela_params != "")
    {
      _params["stela"] = prx::param_loader(stela_params, "");
    }
    _params["planner/environment"] = environment;

    prx::simulation_step = _params["/planner/simulation_step"].as<double>();

    int random_seed{ _params["/planner/random_seed"].as<int>() };
    PARAM_SETUP_WITH_DEFAULT(private_nh, random_seed, random_seed);
    DEBUG_VARS(random_seed);
    prx::init_random(random_seed);

    _tree_topic_name = ros::this_node::getNamespace() + _planner_name + _tree_topic_name;
    _sln_tree_topic_name = ros::this_node::getNamespace() + _planner_name + _sln_tree_topic_name;
    _tree_service_name = ros::this_node::getNamespace() + _planner_name + _tree_service_name;
    _goal_marker_topic_name = ros::this_node::getNamespace() + _planner_name + _goal_marker_topic_name;

    // publishers
    _tree_publisher = private_nh.advertise<prx_models::Tree>(_tree_topic_name, 1, true);
    _sln_tree_publisher = private_nh.advertise<prx_models::Tree>(_sln_tree_topic_name, 1, true);

    _service_filename = private_nh.advertiseService(_tree_service_name, &Derived::tree_service_callback, this);
    _goal_marker_publisher = private_nh.advertise<visualization_msgs::Marker>(_goal_marker_topic_name, 1, true);
  }

  bool tree_service_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    _obstacles = setup_world(_params, _plant, _world_model, _system_group, _collision_group);
    _spec = std::make_shared<PlannerSpecification>(_system_group, _collision_group);
    _query = std::make_shared<PlannerQuery>(_system_group->get_state_space(), _system_group->get_control_space());
    _planner = std::make_shared<Planner>("Planner");

    setup_spec(_params, _spec);
    setup_query(_params, _query, _checker);

    if (_params.exists("planner/medial_axis"))
    {
      prx::param_loader ma_params{ _params["planner/medial_axis"] };
      if (ma_params["use"].as<bool>())
      {
        ma_params["environment"] = _params["planner/environment"].as<>();
        _medial_axis = setup_medial_axis_sampler(ma_params, _query->start_state, _query->goal_state);
        //std::ofstream ofs("/common/home/eg585/prx_ros_ws/data/medial_axis_out.txt");
	//ofs.close();
	
	std::shuffle(_medial_axis.begin(), _medial_axis.end(), prx::global_generator);
        _ma_rate = ma_params["rate"].as<double>();
        _spec->sample_state = [this](prx::space_point_t& s)  // no-lint
        {
          const double rand{ prx::uniform_random(0, 1.0) };
          default_sample_state(s, _system_group->get_state_space());
          
	  if (rand > _ma_rate)
          {
            const int idx{ prx::uniform_int_random(0, _medial_axis.size()) };
            const Eigen::Vector2d pt{ _medial_axis[idx] };
            s->at(0) = pt[0];
            s->at(1) = pt[1];
          }
        };
      }
    }

    DEBUG_VARS(*_spec);
    visualization_msgs::Marker goal_marker{ create_goal_marker() };

    _goal_marker_publisher.publish(goal_marker);

    _planner->link_and_setup_spec(_spec.get());
    _planner->preprocess();
    _planner->link_and_setup_query(_query.get());
    _planner->resolve_query(_checker.get());
    _planner->fulfill_query();

    publish_tree();
    return true;
  }

  visualization_msgs::Marker create_goal_marker()
  {
    visualization_msgs::Marker goal_marker;
    const Eigen::VectorXd goal_state{ Vec(_query->goal_state) };
    goal_marker.header.frame_id = "world";
    goal_marker.header.stamp = ros::Time();
    goal_marker.ns = "goal";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    // TODO: Make this somewhat parametetric to account for different workspaces
    goal_marker.pose.position.x = goal_state[0];
    goal_marker.pose.position.y = goal_state[1];
    goal_marker.pose.position.z = 0;
    goal_marker.pose.orientation.x = 0.0;
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = _query->goal_region_radius * 2;  // Expects diameter
    goal_marker.scale.y = _query->goal_region_radius * 2;  // Expects diameter
    goal_marker.scale.z = _query->goal_region_radius * 2;  // Expects diameter
    goal_marker.color.a = 0.50;                            // Don't forget to set the alpha!
    goal_marker.color.r = 0.50;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.50;
    return goal_marker;
  }

  void publish_tree()
  {
    // DEBUG_VARS(_planner->tree().size(), _planner->tree().num_edges());

    prx::planning::discretize_tree(_planner->tree(), *_planner, _params["/planner/max_edge_duration"].as<double>());
    copy<typename Planner::Node, typename Planner::Edge>(_tree, _planner->tree());
    _tree_publisher.publish(_tree);

    // DEBUG_VARS(_planner->tree().size(), _planner->tree().num_edges());
    if (_params.exists("stela"))
    {
      const prx::param_loader params_stela{ _params["stela"] };
      auto sln_tree = fulfill_stela_query(params_stela, _planner, _query);
      // DEBUG_VARS(sln_tree->size(), sln_tree->num_edges());
      // sln_tree->to_file("/Users/Gary/pracsys/catkin_ws/tree.txt");

      prx_models::Tree sln_ros_tree;
      copy<typename Planner::Node, typename Planner::Edge>(sln_ros_tree, *sln_tree);
      _sln_tree_publisher.publish(sln_ros_tree);
    }
    viz_tree();
  }

  void viz_tree()
  {
    if (_params["/planner/visualize"].as<bool>())
    {
      // _vis_group = std::make_unique<prx::three_js_group_t>(_plant, _obstacles.second);
      _vis_group.reset(new prx::three_js_group_t({ _plant }, { _obstacles.second }));
      const std::string body_name{ _params["/plant/name"].as<>() + "/" + _params["/plant/vis_body"].as<>() };
      auto ss = _system_group->get_state_space();

      _vis_group->add_vis_infos(prx::info_geometry_t::LINE, _query->tree_visualization, body_name, ss);
      _vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, _query->solution_traj, body_name, ss);
      _vis_group->add_animation(_query->solution_traj, ss, _query->start_state);
      _vis_group->output_html("ros_tree.html");
    }
  }

  prx_models::Tree _tree;

  // Topic names
  std::string _goal_marker_topic_name;
  std::string _tree_topic_name;
  std::string _sln_tree_topic_name;
  std::string _tree_service_name;
  std::string _planner_name;

  // Subscribers
  ros::Subscriber _add_node_subscriber;

  // Publishers
  ros::Publisher _tree_publisher;
  ros::Publisher _sln_tree_publisher;
  ros::Publisher _goal_marker_publisher;

  // Services
  ros::ServiceServer _service_filename;

  // ML4KP
  prx::param_loader _params;
  // std::vector<std::shared_ptr<prx::movable_object_t>> _obstacle_list;
  // std::vector<std::string> _obstacle_names;
  prx::PairNameObstacles _obstacles;
  std::shared_ptr<prx::world_model_t> _world_model;
  prx::system_ptr_t _plant;
  std::shared_ptr<prx::system_group_t> _system_group;
  std::shared_ptr<prx::collision_group_t> _collision_group;
  std::shared_ptr<PlannerSpecification> _spec;
  std::shared_ptr<PlannerQuery> _query;
  std::shared_ptr<Planner> _planner;
  std::shared_ptr<prx::condition_check_t> _checker;

  std::unique_ptr<prx::three_js_group_t> _vis_group;  // = new three_js_group_t({ plant }, { obstacle_list });

  std::vector<Eigen::Vector2d> _medial_axis;
  double _ma_rate;
};
}  // namespace motion_planning

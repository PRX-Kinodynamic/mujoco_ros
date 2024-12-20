#pragma once
#include <ros/callback_queue.h>
#include <utils/dbg_utils.hpp>

#include <ros/ros.h>
#include <regex>

#include <prx/utilities/math/first_order_derivative.hpp>
#include <prx/utilities/general/csv_reader.hpp>
#include <prx/utilities/general/type_conversions.hpp>

namespace utils
{

// SDF class, because inefficiency has become the norm...
class signed_distance_field_t
{
  using State = Eigen::Vector2d;
  using Rotation = Eigen::Matrix3d;
  using Translation = Eigen::Vector3d;
  using CollisionInfo = prx::fg::collision_info_t;
  using CollisionInfoPtr = std::shared_ptr<CollisionInfo>;
  using IdxPair = std::pair<std::size_t, std::size_t>;

  struct configuration_from_state
  {
    void operator()(Rotation& rotation, Translation& translation, const State& state)
    {
      rotation = Rotation::Identity();
      translation[0] = state[0];
      translation[1] = state[1];
      translation[2] = 0;
    }
    void operator()(const bool collision, const State& state, const Translation& p1, const Translation& p2,
                    Eigen::MatrixXd& H)
    {
      prx_throw("Not implemented");
    }
  };

  signed_distance_field_t()
  {
  }

  bool from_file()
  {
    using csv_reader_t = prx::utilities::csv_reader_t;
    using prx::utilities::convert_to;

    bool success{ false };
    if (std::filesystem::exists(_file))
    {
      PRINT_MSG("SDF using file: " + _file);

      csv_reader_t reader(_file, ' ');
      // First line has the parameters
      auto line = reader.next_line();
      _min_bound = State{ convert_to<double>(line[1]), convert_to<double>(line[2]) };
      _max_bound = State{ convert_to<double>(line[3]), convert_to<double>(line[4]) };
      _resolution = convert_to<double>(line[5]);
      // DEBUG_VARS(_min_bound.transpose(), _max_bound.transpose(), _resolution);
      init_matrices();
      std::size_t x, y;
      double dist, dx, dy;
      std::size_t total{ 0 };
      while (reader.has_next_line())
      {
        auto line = reader.next_line();
        if (line.size() == 0)
          continue;
        dist = convert_to<double>(line[2]);
        dx = convert_to<double>(line[3]);
        dy = convert_to<double>(line[4]);
        x = convert_to<std::size_t>(line[5]);
        y = convert_to<std::size_t>(line[6]);
        // DEBUG_VARS(line);
        // DEBUG_VARS(x, y, idxs.first, idxs.second);
        // DEBUG_VARS(dist, dx, dy);
        _sdf(x, y) = dist;
        _sdf_dx(x, y) = dx;
        _sdf_dy(x, y) = dy;
        total++;
      }
      const long expected_values{ _sdf.rows() * _sdf.cols() };
      // DEBUG_VARS(total, expected_values);
      success = (total == expected_values);
      if (not success)
      {
        PRINT_MSG("Failed to read SDF from file.");
      }
    }
    return success;
  }

  virtual void init(const prx::param_loader& params)
  {
    bool initialized{ false };
    if (params.exists("geometry"))
    {
      _robot_collision_info = std::make_shared<prx::fg::collision_info_t>(params["geometry"]);
    }
    _resolution = params.exists("resolution") ? params["resolution"].as<double>() : 0.0;
    _environment = params.exists("environment") ? params["environment"].as<std::string>() : "None";

    if (params.exists("directory"))
    {
      const std::string dir{ params.exists("directory") ? params["directory"].as<std::string>() : "None" };
      std::filesystem::path path(_environment);
      _file = dir + "/" + path.stem().string() + ".txt";
      initialized = from_file();
    }
    if (not initialized)
    {
      create_sdf();
    }
  }

  // Need to adapt for 2D maps: Increase Z, cylinders instead of spheres, etc.
  void adapt_for_2d(prx::geometry_type_t& g_type, std::vector<double>& params)
  {
    switch (g_type)
    {
      case prx::geometry_type_t::BOX:
        // Increase Z to avoid a collision or distance computation with Z.
        params[2] = 10 * std::max(params[0], params[1]);
        break;
      case prx::geometry_type_t::SPHERE:
        params = std::vector<double>({ params[0], 10 * params[0] });
        g_type = prx::geometry_type_t::CYLINDER;
        break;
      case prx::geometry_type_t::CYLINDER:
        params[1] = 10 * params[0];
        break;
      default:
        prx_throw("Shape not supported");
    };
  }

  inline void init_matrices()
  {
    const double xdiff{ _max_bound[0] - _min_bound[0] };
    const double ydiff{ _max_bound[1] - _min_bound[1] };
    const std::size_t w{ static_cast<std::size_t>(std::ceil(xdiff / _resolution + 1)) };
    const std::size_t h{ static_cast<std::size_t>(std::ceil(ydiff / _resolution + 1)) };
    _sdf = Eigen::MatrixXd(w, h);
    _sdf_dx = Eigen::MatrixXd(w, h);
    _sdf_dy = Eigen::MatrixXd(w, h);
  }

  void create_sdf()
  {
    using ObstacleFactor = prx::fg::obstacle_factor_t<State, configuration_from_state>;
    using MatrixXb = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;
    // DEBUG_VARS(_environment);
    auto prx_obstacles = prx::load_obstacles(_environment);
    std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ prx_obstacles.second };
    std::vector<std::string> obstacle_names{ prx_obstacles.first };
    const prx::EnvironmentBounds bounds{ prx::obstacle_loader_t::bounds_from_yaml(_environment) };

    std::vector<CollisionInfoPtr> obstacles{};
    for (auto obstacle : obstacle_list)
    {
      prx::movable_object_t::Geometries geometries{ obstacle->get_geometries() };
      prx::movable_object_t::Configurations configurations{ obstacle->get_configurations() };

      const std::size_t total_geoms{ geometries.size() };

      for (int i = 0; i < total_geoms; ++i)
      {
        std::shared_ptr<prx::geometry_t> g{ geometries[i].second };
        std::shared_ptr<prx::transform_t> tf{ configurations[i].second };

        prx::geometry_type_t g_type{ g->get_geometry_type() };
        std::vector<double> g_params{ g->get_geometry_params() };

        const Rotation rot{ tf->rotation() };
        const Translation t{ tf->translation() };
        adapt_for_2d(g_type, g_params);

        CollisionInfoPtr obstacle{ std::make_shared<prx::fg::collision_info_t>(g_type, g_params, rot, t) };

        obstacles.push_back(obstacle);
      }
    }

    // using EnvironmentBounds = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
    const double& xmin{ bounds.first[0] };
    const double& ymin{ bounds.first[1] };
    const double& xmax{ bounds.second[0] };
    const double& ymax{ bounds.second[1] };
    _min_bound = State{ xmin, ymin };
    _max_bound = State{ xmax, ymax };
    // const double xdiff{ bounds.second[0] - bounds.first[0] };
    // const double ydiff{ bounds.second[1] - bounds.first[1] };
    init_matrices();
    const std::size_t w{ static_cast<std::size_t>(_sdf.cols()) };
    const std::size_t h{ static_cast<std::size_t>(_sdf.rows()) };

    configuration_from_state cfs;
    ObstacleFactor::CollideResult collision_result;
    ObstacleFactor::DistanceResult distance_result;
    Translation pt, p1, p2;
    State position{ State::Zero() };

    double inside_dist{ 0.0 };
    for (std::size_t x = 0; x < w; ++x)
    {
      position[0] = _min_bound[0] + x * _resolution;
      int sign{ 1 };
      bool prev_collision{ false };
      bool first{ true };
      for (std::size_t y = 0; y < h; ++y)
      {
        position[1] = _min_bound[1] + y * _resolution;
        bool inside{ false };
        double dist{ std::numeric_limits<double>::max() };
        for (auto obs : obstacles)
        {
          // collision = ObstacleFactor::in_collision(position, obs, _robot_collision_info, cfs, collision_result, pt);
          inside = ObstacleFactor::inside_obstacle(position, obs, cfs, inside_dist);
          const double obs_dist{ ObstacleFactor::distances(position, p1, p2, obs, _robot_collision_info, cfs,
                                                           distance_result) };

          if (inside)
          {
            dist = -inside_dist;
            break;
          }
          dist = std::min(dist, obs_dist);
        }

        _sdf(x, y) = dist;
      }
    }

    using OutType = Eigen::Vector<double, 1>;
    using SdfWrapper = std::function<OutType(const Eigen::Vector2d&)>;
    using Derivative = prx::math::first_order_derivative_t<SdfWrapper, Eigen::Vector2d, 3>;

    SdfWrapper wrapper = [this](const Eigen::Vector2d& p) { return OutType(distance(p)); };

    Derivative derivative(wrapper, _resolution);
    Eigen::Vector2d H;
    for (std::size_t x = 0; x < w; ++x)
    {
      position[0] = _min_bound[0] + x * _resolution;
      for (std::size_t y = 0; y < h; ++y)
      {
        position[1] = _min_bound[1] + y * _resolution;
        H = derivative(position);
        _sdf_dx(x, y) = H[0];
        _sdf_dy(x, y) = H[1];
      }
    }
  }

  inline IdxPair coordinates_to_indices(const double& x, const double& y) const
  {
    const double x_p{ std::min(std::max(x, _min_bound[0]), _max_bound[0]) };
    const double y_p{ std::min(std::max(y, _min_bound[1]), _max_bound[1]) };
    const std::size_t x_idx{ static_cast<std::size_t>(std::ceil((x_p - _min_bound[0]) / _resolution)) };
    const std::size_t y_idx{ static_cast<std::size_t>(std::ceil((y_p - _min_bound[1]) / _resolution)) };
    return { x_idx, y_idx };
  }

public:
  signed_distance_field_t(const prx::param_loader& params)
  {
    init(params);
  }

  static std::shared_ptr<signed_distance_field_t> create(const prx::param_loader& params)
  {
    return std::make_shared<signed_distance_field_t>(params);
  }

  virtual ~signed_distance_field_t() {};

  static prx::param_loader default_parameters()
  {
    // prx::param_loader& params
    // params["geometry"].set("SPHERE");
    prx::param_loader params;
    params["geometry"] = CollisionInfo::default_parameters();
    params["resolution"].set(0.1);
    params["environment"].set("environments/empty.yaml");
    params["directory"].set("/tmp");
    return params;
  }

  double distance(const double& x, const double& y) const
  {
    const IdxPair idxs{ coordinates_to_indices(x, y) };
    return _sdf(idxs.first, idxs.second);
  }

  Eigen::Vector2d jacobian(const double& x, const double& y) const
  {
    const IdxPair idxs{ coordinates_to_indices(x, y) };
    const double dx{ _sdf_dx(idxs.first, idxs.second) };
    const double dy{ _sdf_dy(idxs.first, idxs.second) };
    return Eigen::Vector2d(dx, dy);
  }

  double distance(const Eigen::Vector2d& x) const
  {
    return distance(x[0], x[1]);
  }

  Eigen::Vector2d jacobian(const Eigen::Vector2d& x) const
  {
    return jacobian(x[0], x[1]);
  }

  void to_file() const
  {
    std::ofstream ofs(_file.c_str());
    State xt;

    ofs << "# ";
    ofs << _min_bound.transpose() << " ";
    ofs << _max_bound.transpose() << " ";
    ofs << _resolution << " ";
    ofs << "\n";

    for (std::size_t x = 0; x < _sdf.cols(); ++x)
    {
      xt[0] = _min_bound[0] + x * _resolution;
      for (std::size_t y = 0; y < _sdf.rows(); ++y)
      {
        xt[1] = _min_bound[1] + y * _resolution;
        const double dist{ _sdf(x, y) };
        const double dx{ _sdf_dx(x, y) };
        const double dy{ _sdf_dy(x, y) };
        ofs << xt[0] << " " << xt[1] << " " << dist << " ";
        ofs << dx << " " << dy << " ";
        ofs << x << " " << y << " ";
        ofs << "\n";
      }
    }
    ofs.close();
  }

protected:
  Eigen::MatrixXd _sdf;
  Eigen::MatrixXd _sdf_dx;
  Eigen::MatrixXd _sdf_dy;

  State _min_bound;
  State _max_bound;
  double _resolution;

  std::string _environment;
  std::string _file;

  CollisionInfoPtr _robot_collision_info;
};

}  // namespace utils

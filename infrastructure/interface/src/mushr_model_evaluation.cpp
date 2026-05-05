#include <thread>
#include <unordered_map>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <ml4kp_bridge/defs.h>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <interface/SetDuration.h>
#include <prx/utilities/general/csv_reader.hpp>
#include <utils/rosparams_utils.hpp>
#include <utils/dbg_utils.hpp>
#include <ml4kp_bridge/product_lie_group.hpp>
#include <utils/std_utils.hpp>

#include <gtsam/geometry/Pose2.h>
#include <prx_models/Graph.h>

using prx::utilities::convert_to;
using CsvReader = prx::utilities::csv_reader_t;

using Pose = gtsam::Pose2;
using Velocity = Eigen::Vector3d;
using State = gtsam::ProductLieGroupV43<Pose, Velocity>;
using Trajectory = std::vector<State>;
struct model_error_t
{
  double position_error;
  double orientation_error;
  double pose_error;
  double linear_velocity_error;
  double angular_velocity_error;
  double velocity_error;
  double state_error;
  std::size_t total;

  model_error_t()
    : position_error(0.)
    , orientation_error(0.)
    , pose_error(0.)
    , linear_velocity_error(0.)
    , angular_velocity_error(0.)
    , velocity_error(0.)
    , state_error(0.)
    , total(0) {};

  model_error_t(const Eigen::VectorXd error)
    : position_error(error.head(2).norm())
    , orientation_error(error[2])
    , pose_error(error.head(3).norm())
    , linear_velocity_error(error.segment<2>(3).norm())
    , angular_velocity_error(error[5])
    , velocity_error(error.tail(3).norm())
    , state_error(error.norm())
    , total(1)
  {
  }
  model_error_t& operator+=(const model_error_t& other)
  {
    this->position_error += other.position_error;
    this->orientation_error += other.orientation_error;
    this->pose_error += other.pose_error;
    this->linear_velocity_error += other.linear_velocity_error;
    this->angular_velocity_error += other.angular_velocity_error;
    this->velocity_error += other.velocity_error;
    this->state_error += other.state_error;
    this->total += other.total;
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, const model_error_t& obj)
  {
    os << obj.position_error << " ";
    os << obj.orientation_error << " ";
    os << obj.pose_error << " ";
    os << obj.linear_velocity_error << " ";
    os << obj.angular_velocity_error << " ";
    os << obj.velocity_error << " ";
    os << obj.state_error << " ";
    os << obj.total << " ";
    return os;
  }

  static std::string header()
  {
    std::stringstream strstr;
    strstr << "# ";
    strstr << "position_error ";
    strstr << "orientation_error ";
    strstr << "pose_error ";
    strstr << "linear_velocity_error ";
    strstr << "angular_velocity_error ";
    strstr << "velocity_error ";
    strstr << "state_error ";
    strstr << "total";
    strstr << "\n";
    return strstr.str();
  }
};

std::string fix_string(const double dt)
{
  static char* time_string = new char[100];
  snprintf(time_string, 100, "%.1f", dt);

  return std::string(time_string);
}

struct query_t
{
  std::map<int, Trajectory> trajectories;
  std::vector<model_error_t> errors;
  // ros::Subscriber subscriber;
  ros::Subscriber request_subscriber, evaluation_subscriber;
  ros::Publisher response_publisher;
  std::shared_ptr<CsvReader> csv_reader;
  int next_id;

  std::vector<std::ofstream> ofs;
  std::vector<std::string> filenames;
  std::string avg_filename;
  CsvReader::Block<double> current_block;

  query_t(const std::string output_dir) : errors(11), ofs(11), next_id(0)
  {
    const std::string timestamp{ utils::timestamp() };
    for (int i = 0; i < 11; ++i)
    {
      std::stringstream strstr;
      strstr << output_dir << "/errors_";
      strstr << std::setfill('0') << std::setw(2) << i << "_";
      strstr << timestamp;
      strstr << ".txt";
      ofs[i].open(strstr.str());
      filenames.push_back(strstr.str());

      ofs[i] << model_error_t::header();
    }
    avg_filename = output_dir + "/avg_errors_" + timestamp + ".txt";
  }

  ~query_t()
  {
  }

  void close()
  {
    std::ofstream avg_ofs(avg_filename.c_str());

    for (int i = 0; i < 11; ++i)
    {
      avg_ofs << i << " " << errors[i] << "\n";
      ofs[i].close();
    }
    avg_ofs.close();
  }

  void to_file(std::vector<model_error_t>& errors)
  {
    for (int i = 0; i < 11; ++i)
    {
      ofs[i] << errors[i] << std::endl;
    }
  }

  CsvReader::Block<double> remove_comments(CsvReader::Block<std::string>& block)
  {
    CsvReader::Block<double> aux_block;
    for (auto& line : block)
    {
      if (line.size() == 0)
        continue;
      if (line[0][0] == '#')  // comment line
        continue;

      aux_block.emplace_back();
      for (auto& e : line)
      {
        aux_block.back().push_back(convert_to<double>(e));
      }
    }
    // DEBUG_VARS(aux_block)
    return aux_block;
  }

  std::tuple<Trajectory, prx::param_loader> subblock_to_traj()
  {
    prx::param_loader param;
    Trajectory traj;
    double ti{ 0.0 };
    const std::string idx{ convert_to<std::string>(next_id) };
    param[idx + "/id"].set(next_id);
    param[idx + "/controls"].set(std::vector<double>());

    const std::vector<double> x0 = { current_block[0][1], current_block[0][2], current_block[0][3],
                                     current_block[0][7], current_block[0][8], current_block[0][9] };
    param[idx + "/x0"].set(x0);

    for (int i = 0; i <= 10; ++i)
    {
      //    0  1  2. 3. 4. 5. 6.    7.   8.     9.    10.   11.   12.   13 14
      // # ti x0 y0 th0 x1 y1 th1 xdot0 ydot0 thdot0 xdot1 ydot1 thdot1 u0 u1
      const gtsam::Pose2 pose(current_block[i][1], current_block[i][2], current_block[i][3]);
      const Eigen::Vector3d vel(current_block[i][7], current_block[i][8], current_block[i][9]);
      traj.emplace_back(pose, vel);

      const std::vector<double> ctrl = { current_block[i][13], current_block[i][14] };

      param[idx + "/controls"][fix_string(ti)].set(ctrl);
      ti += 0.1;
    }
    current_block.erase(current_block.begin());

    trajectories[next_id] = traj;
    // DEBUG_VARS(trajectories.size())
    next_id++;
    return { traj, param };
  }

  std::optional<std::string> trajectory_data()
  {
    if (current_block.size() > 11)
    {
      prx::param_loader param_out;
      auto [traj, param] = subblock_to_traj();
      // param_out["id"].set(next_id);
      // param_out["trajectory"] = param;

      std::stringstream strstr;
      strstr << param;
      // DEBUG_VARS(param);
      // DEBUG_VARS(strstr.str());
      return strstr.str();
    }

    if (csv_reader->has_next_line())
    {
      auto block = csv_reader->next_block();
      current_block = remove_comments(block);
      return trajectory_data();
    }

    return {};
  }

  void init_csv(const std::string filename)
  {
    csv_reader = std::make_shared<CsvReader>(filename);
  }

  void add(const std::vector<model_error_t>& new_errors)
  {
    for (int i = 0; i < errors.size(); ++i)
    {
      errors[i] += new_errors[i];
    }
  }
};

struct evaluator_t
{
  ros::NodeHandle _nh;
  std::string _output_directory;
  std::vector<std::string> _data_files;
  std::map<std::string, std::string> _datasets;

  ros::Subscriber _initialization_subscriber;
  ros::Publisher _error_publisher;

  // std::vector<ros::Subscriber> _subscribers;
  std::vector<ros::Publisher> _publishers;

  std::vector<std::shared_ptr<query_t>> _queries;

  evaluator_t(ros::NodeHandle& nh) : _nh(nh)
  {
    std::string& output_directory{ _output_directory };
    std::map<std::string, std::string>& datasets{ _datasets };

    PARAM_SETUP(nh, datasets);
    PARAM_SETUP(nh, output_directory);

    _initialization_subscriber = nh.subscribe("/ModelEvaluation/initialize", 1, &evaluator_t::init_callback, this);
    _error_publisher = nh.advertise<std_msgs::String>("/ModelEvaluation/error", 1);

    for (auto pair : datasets)
    {
      const std::string dataset{ pair.first };
      const std::string filename{ pair.second };
      DEBUG_VARS(dataset, filename)
    }
    // TODO: help topic
  }

  ~evaluator_t()
  {
    for (auto q : _queries)
    {
      if (q)
      {
        q->close();
        q = nullptr;
      }
    }
  }

  void publish_error(const std::string error)
  {
    std_msgs::String msg;
    std::stringstream strstr;
    strstr << "[ " << ros::Time::now() << " ] ";
    strstr << error;
    msg.data = strstr.str();
    _error_publisher.publish(msg);
  }

  void init_callback(const std_msgs::StringConstPtr msg)
  {
    // DEBUG_VARS(msg->data)
    prx::param_loader params;
    params.from_string(msg->data);
    if (not params.exists("topic_ns"))
    {
      publish_error("Initialization: 'topic_ns' not found.");
      return;
    }
    if (not params.exists("data_set"))
    {
      publish_error("Initialization: 'data_set' not found.");
      return;
    }
    const std::string topic_ns{ params["topic_ns"].as<>() };
    const std::string data_set{ params["data_set"].as<>() };

    const std::string request_topic{ topic_ns + "/request" };
    const std::string response_topic{ topic_ns + "/response" };
    const std::string evaluate_topic{ topic_ns + "/evaluate" };
    auto next_query = std::make_shared<query_t>(_output_directory);
    init_dataset(data_set, next_query);

    next_query->response_publisher = _nh.advertise<std_msgs::String>(response_topic, 1, true);
    next_query->request_subscriber = _nh.subscribe<std_msgs::String>(
        request_topic, 1, boost::bind(&evaluator_t::request_callback, this, _1, next_query));
    next_query->evaluation_subscriber = _nh.subscribe<std_msgs::String>(
        evaluate_topic, 1, boost::bind(&evaluator_t::evaluation_callback, this, _1, next_query));

    _queries.push_back(next_query);
    DEBUG_VARS(request_topic)
    DEBUG_VARS(response_topic)
    DEBUG_VARS(evaluate_topic)

    prx::param_loader init_params;
    std_msgs::String init_msg;
    init_params["header"].set("initialized");
    init_params["files"].set(next_query->filenames);

    // std::string
    init_msg.data = std::string(init_params);
    next_query->response_publisher.publish(init_msg);
  }

  void init_dataset(std::string data_set, std::shared_ptr<query_t> query)
  {
    std::transform(data_set.begin(), data_set.end(), data_set.begin(), ::toupper);
    if (_datasets.count(data_set) == 0)
    {
      publish_error("[Initialization] data_set '" + data_set + "' not found.");
      return;
    }
    const std::string filename{ _datasets[data_set] };

    query->init_csv(filename);
  }

  // Trajectories of up to 1 sec duration. Assuming 0.1 between states:
  // 0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0
  //  0 - 1 - 2 - 3 - 4 - 5 - 6 - 7 - 8 - 9 - 10

  void request_callback(const std_msgs::StringConstPtr msg, std::shared_ptr<query_t> query)
  {
    prx::param_loader params;
    params.from_string(msg->data);
    // DEBUG_VARS(params)
    if (not params.exists("batch_size"))
    {
      publish_error("Request: no 'batch_size' parameter.");
      return;
    }
    const int batch_size{ params["batch_size"].as<int>() };

    std_msgs::String out_msg;
    out_msg.data = "header: data\n";
    // out_msg.data = ": data\n";
    // prx::param_loader params_out;
    bool finished{ false };
    for (int i = 0; i < batch_size; ++i)
    {
      auto data = query->trajectory_data();
      if (data)
      {
        out_msg.data += "\n";
        out_msg.data += *data;
      }
      else
      {
        finished = true;
        query->csv_reader = nullptr;
        break;
      }
    }
    out_msg.data += "\nfinished: " + convert_to<std::string>(finished) + "\n";
    // DEBUG_VARS(out_msg.data)

    query->response_publisher.publish(out_msg);
  }

  // void trajectories_callback(const std_msgs::StringConstPtr msg, query_t& query)
  void evaluation_callback(const std_msgs::StringConstPtr msg, std::shared_ptr<query_t> query)
  {
    if (query == nullptr)
    {
      publish_error("[evaluation_callback] invalid query");
    }

    prx::param_loader params;
    params.from_string(msg->data);
    // auto trajs_in = params["trajectories"];
    // DEBUG_VARS(params)
    std::vector<std::string> keys{ params.keys() };
    // DEBUG_VARS(query->next_id)
    for (auto key : keys)
    {
      prx::param_loader param_traj{ params[key]["trajectory"] };
      const int id{ param_traj["id"].as<int>() };
      const Trajectory traj{ param_to_trajectory(param_traj) };

      if (query->trajectories.count(id) == 0)
      {
        publish_error("Received trajectory's id: " + convert_to<std::string>(id) + " unknown");
        continue;
      }
      const Trajectory traj_gt{ query->trajectories[id] };
      std::vector<model_error_t> errors{ trajectory_error(traj_gt, traj) };
      query->trajectories.erase(query->trajectories.find(id));

      if (errors.size() != 11)
      {
        publish_error("Received trajectory's errors: are not the correct size");
      }
      for (int i = 0; i < 11; ++i)
      {
        query->errors[i] += errors[i];
      }
      query->to_file(errors);
    }

    if (query->csv_reader == nullptr and query->trajectories.size() == 0)
    {
      query->close();
      query->request_subscriber.shutdown();
      query->evaluation_subscriber.shutdown();
      query->response_publisher.shutdown();
      query = nullptr;
    }
  }

  Trajectory param_to_trajectory(const prx::param_loader traj_as_param)
  {
    Trajectory traj;
    // DEBUG_VARS(traj_as_param)
    for (double i = 0; i <= 1.0; i += 0.1)
    {
      const std::string ti{ fix_string(i) };
      if (traj_as_param.exists(ti))
      {
        Eigen::Vector<double, 6> xi{ traj_as_param[ti].as<Eigen::Vector<double, 6>>() };
        traj.emplace_back(gtsam::Pose2(xi[0], xi[1], xi[2]), xi.tail(3));
      }
      else
      {
        publish_error("TrajectoryError: The id " + ti + " was not present in the input trajectory.");
      }
    }
    // DEBUG_VARS(traj)
    return traj;
  }

  std::vector<model_error_t> trajectory_error(const Trajectory x_gt, const Trajectory x_hat)
  {
    // DEBUG_VARS(x_gt.size())
    // DEBUG_VARS(x_hat.size())
    if (x_gt.size() != x_hat.size())
    {
      publish_error("TrajectoryError: trajectories sizes mismatch.");
      return {};
    }
    std::vector<model_error_t> errors;
    for (int i = 0; i < x_gt.size(); ++i)
    {
      errors.push_back(state_error(x_gt[i], x_hat[i]));
    }
    return errors;
  }

  model_error_t state_error(const State x_gt, const State x_hat)
  {
    const State btw{ gtsam::traits<State>::Between(x_gt, x_hat) };
    const Eigen::VectorXd error{ gtsam::traits<State>::Logmap(btw) };
    // DEBUG_VARS(error.transpose());
    return model_error_t(error);
  }
};

int main(int argc, char** argv)
{
  const std::string node_name{ "ModelEvaluation" };
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  evaluator_t evaluator(nh);

  ros::spin();
}
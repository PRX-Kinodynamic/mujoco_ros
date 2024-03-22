#pragma once
#include "defs.h"
#include <sstream>
#include <cstdio>
#include <mutex>
#include <thread>

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <ml4kp_bridge/defs.h>

namespace mj_ros
{
class simulator_t;
class simulator_visualizer_t;

using SimulatorPtr = std::shared_ptr<simulator_t>;
using VisualizerPtr = std::shared_ptr<simulator_visualizer_t>;

class simulator_t : public std::enable_shared_from_this<simulator_t>
{
private:
  bool save_trajectory;
  bool visualize;
  std::vector<std::vector<double>> trajectory;
  std::vector<double> current_state;

  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjrContext con;

  GLFWwindow* window;

  simulator_t(const std::string& model_path, bool _save_trajectory)
  {
    std::cout << model_path << std::endl;
    std::string error;
    error.reserve(1000);
    m = mj_loadXML(model_path.c_str(), NULL, error.data(), error.capacity());
    if (!m or error.size() != 0)
    {
      std::cerr << "Error in loading model." << std::endl;
      std::cout << error << std::endl;
    }

    d = mj_makeData(m);
    std::cout << "Simulation timestep: " << m->opt.timestep << std::endl;
    ROS_ASSERT(m->opt.timestep > 0);
    for (int i = 0; i < 1.0 / m->opt.timestep; i++)
    {
      mj_step(m, d);
    }
    for (int i = 0; i < m->nq; i++)
    {
      std::cout << d->qpos[i] << " ";
    }
    std::cout << std::endl;
    save_trajectory = _save_trajectory;
    if (save_trajectory)
    {
      add_current_state_to_trajectory();
    }
  }

public:
  std::mutex _mj_reset_mutex;
  mjModel* m;
  mjData* d;

  [[nodiscard]] static std::shared_ptr<simulator_t> initialize(const std::string model_path, const bool save_trajectory)
  {
    return std::shared_ptr<simulator_t>(new simulator_t(model_path, save_trajectory));
  }

  ~simulator_t()
  {
    mj_deleteData(d);
    mj_deleteModel(m);
    if (visualize)
    {
      mjr_freeContext(&con);
      mjv_freeScene(&scn);
      glfwTerminate();
    }
  }

  void set_control(const std::vector<double>& control)
  {
    if (control.size() != m->nu)
      std::cerr << "Control size does not match." << std::endl;

    for (int i = 0; i < m->nu; i++)
    {
      d->ctrl[i] = control[i];
    }
  }

  void propagate(const double duration)
  {
    int num_steps = duration / m->opt.timestep;
    for (int i = 0; i < num_steps; i++)
    {
      this->step_simulation();
    }
  }

  void run()
  {
    ros::Rate rate(1.0 / m->opt.timestep);
    while (ros::ok())
    {
      step_simulation();
      rate.sleep();
    }
  }

  void step_simulation()
  {
    for (int i = 0; i < m->nv; i++)
    {
      d->qacc_warmstart[i] = 0;
    }
    _mj_reset_mutex.lock();
    mj_step(m, d);
    _mj_reset_mutex.unlock();
    if (save_trajectory)
    {
      add_current_state_to_trajectory();
    }
  }

  void add_current_state_to_trajectory()
  {
    current_state.clear();
    for (int i = 0; i < m->nq; i++)
    {
      current_state.push_back(d->qpos[i]);
    }
    for (int i = 0; i < m->nv; i++)
    {
      current_state.push_back(d->qvel[i]);
    }
    trajectory.push_back(current_state);
  }

  std::string print_trajectory(const int precision = 4)
  {
    std::stringstream out(std::stringstream::out);
    out << std::fixed << std::setprecision(precision);
    for (int i = 0; i < trajectory.size(); i++)
    {
      for (int j = 0; j < trajectory[i].size(); j++)
      {
        out << trajectory[i][j] << ",";
      }
      out << std::endl;
    }
    return out.str();
  }

  void reset_simulation(const std_msgs::Empty::ConstPtr& msg)
  {
    _mj_reset_mutex.lock();

    ROS_INFO("Resetting simulation.");
    reset_simulation();

    _mj_reset_mutex.unlock();
  }

  void reset_simulation()
  {
    mj_resetData(m, d);
    if (save_trajectory)
    {
      trajectory.clear();
      add_current_state_to_trajectory();
    }
  }
};

class simulator_visualizer_t
{
public:
  simulator_visualizer_t(SimulatorPtr& sim) : _sim(sim)
  {
    button_left = button_middle = button_right = false;
    lastx = lasty = 0;
    if (!glfwInit())
      std::cerr << "Error in initializing GLFW." << std::endl;

    window = glfwCreateWindow(1200, 900, "MuJoCo", NULL, NULL);
    if (!window)
      std::cerr << "Error in creating GLFW window." << std::endl;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(_sim->m, &scn, 1000);
    mjr_makeContext(_sim->m, &con, mjFONTSCALE_150);

    glfwSetWindowUserPointer(window, this);
    glfwSetCursorPosCallback(window, [](GLFWwindow* window, double xpos, double ypos) {
      auto sim = static_cast<simulator_visualizer_t*>(glfwGetWindowUserPointer(window));
      sim->mouse_move(window, xpos, ypos);
    });
    glfwSetMouseButtonCallback(window, [](GLFWwindow* window, int button, int action, int mods) {
      auto sim = static_cast<simulator_visualizer_t*>(glfwGetWindowUserPointer(window));
      sim->mouse_button(window, button, action, mods);
    });
    glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
      auto sim = static_cast<simulator_visualizer_t*>(glfwGetWindowUserPointer(window));
      sim->scroll(window, xoffset, yoffset);
    });
  }

  static VisualizerPtr initialize(SimulatorPtr& sim, const bool viz = true)
  {
    if (viz)
      return std::make_shared<simulator_visualizer_t>(sim);
    return nullptr;
  }

  void draw_goal()
  {
    if (goal_position.size() != 0)
    {
      mjvGeom* goal_geom = scn.geoms + scn.ngeom++;
      mjv_initGeom(goal_geom, mjGEOM_SPHERE, NULL, NULL, NULL, NULL);
      goal_geom->rgba[0] = 0.0;
      goal_geom->rgba[1] = 1.0;
      goal_geom->rgba[2] = 0.0;
      goal_geom->rgba[3] = 0.5;
      goal_geom->size[0] = goal_radius;
      goal_geom->size[1] = goal_radius;
      goal_geom->size[2] = goal_radius;
      goal_geom->pos[0] = goal_position[0];
      goal_geom->pos[1] = goal_position[1];
      goal_geom->pos[2] = 0.0;
    }
  }

  void draw_trajectory()
  {
    if (trajectory_to_visualize.size() > 0)
    {
      mjvGeom* start_geom = scn.geoms + scn.ngeom++;
      mjv_initGeom(start_geom, mjGEOM_SPHERE, NULL, NULL, NULL, NULL);
      start_geom->rgba[0] = 1.0;
      start_geom->rgba[1] = 0.0;
      start_geom->rgba[2] = 0.0;
      start_geom->rgba[3] = 0.5;
      start_geom->size[0] = 0.05;
      start_geom->size[1] = 0.05;
      start_geom->size[2] = 0.05;
      start_geom->pos[0] = trajectory_to_visualize[0][0];
      start_geom->pos[1] = trajectory_to_visualize[0][1];
      start_geom->pos[2] = lineheight;
      for (int i = line_step; i < trajectory_to_visualize.size(); i += line_step)
      {
        if (scn.ngeom >= scn.maxgeom)
        {
          ROS_WARN("Max geom reached.");
          break;
        }
        mjvGeom* line_geom = scn.geoms + scn.ngeom++;
        mjv_initGeom(line_geom, mjGEOM_LINE, NULL, NULL, NULL, linecolor);
        mjv_makeConnector(line_geom, mjGEOM_LINE, linewidth, trajectory_to_visualize[i - line_step][0],
                          trajectory_to_visualize[i - line_step][1], lineheight, trajectory_to_visualize[i][0],
                          trajectory_to_visualize[i][1], lineheight);
      }
    }
  }

  void run()
  {
    ros::WallRate r(30);
    mjrRect viewport{ 0, 0, 1200, 900 };

    while (ros::ok() and !glfwWindowShouldClose(window))
    {
      _sim->_mj_reset_mutex.lock();

      glfwMakeContextCurrent(window);
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
      mjv_updateScene(_sim->m, _sim->d, &opt, NULL, &cam, mjCAT_ALL, &scn);

      draw_goal();
      draw_trajectory();

      mjr_render(viewport, &scn, &con);
      snprintf(time_string, 100, "Sim time: = %f", _sim->d->time);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, time_string, nullptr, &con);
      glfwSwapBuffers(window);
      glfwPollEvents();

      _sim->_mj_reset_mutex.unlock();

      r.sleep();
    }
    ros::shutdown();
  }

  inline void set_goal_pos(const geometry_msgs::Pose2D::ConstPtr& msg)
  {
    goal_position = { msg->x, msg->y };
  }

  inline void set_goal_radius(const std_msgs::Float64::ConstPtr& msg)
  {
    goal_radius = msg->data;
  }

  inline void set_trajectory_to_visualize(const ml4kp_bridge::TrajectoryStampedConstPtr& message)
  {
    const ml4kp_bridge::Trajectory& msg{ message->trajectory };
    trajectory_to_visualize.clear();
    const std::vector<ml4kp_bridge::SpacePoint>& points = msg.data;
    for (const auto& point : points)
    {
      trajectory_to_visualize.emplace_back(point.point);
    }
  }

  inline void reset(const std_msgs::Empty::ConstPtr& msg)
  {
    goal_position.clear();
    goal_radius = 0.0;
    trajectory_to_visualize.clear();
  }

protected:
  mjvScene scn;
  mjrContext con;
  mjvCamera cam;
  mjvOption opt;
  GLFWwindow* window;
  std::shared_ptr<simulator_t> _sim;

  std::vector<double> goal_position;
  double goal_radius;

  std::vector<std::vector<double>> trajectory_to_visualize;

  char* time_string = new char[100];

  bool button_left, button_right, button_middle;
  double lastx, lasty;

  float linecolor[4] = { 1.0, 0.0, 0.0, 1.0 };
  double linewidth = 3.0;
  double lineheight = 0.1;
  double line_step = 10;

  void mouse_button(GLFWwindow* window, int button, int act, int mods)
  {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
  }
  void mouse_move(GLFWwindow* window, double xpos, double ypos)
  {
    if (!button_left && !button_middle && !button_right)
    {
      return;
    }
    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;
    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
    {
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (button_left)
    {
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
      action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(_sim->m, action, dx / height, dy / height, &scn, &cam);
  }
  void scroll(GLFWwindow* window, double xoffset, double yoffset)

  {
    mjv_moveCamera(_sim->m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
  }
};

template <class... RunnableObjects, std::enable_if_t<sizeof...(RunnableObjects) == 0, bool> = true>
void run_thread(std::vector<std::thread>& threads, RunnableObjects&... runnable_objects)
{
}

template <class First, class... RunnableObjects>
inline void run_thread(std::vector<std::thread>& threads, First& first, RunnableObjects&... runnable_objects)
{
  threads.emplace_back(&First::run, &first);
  run_thread<RunnableObjects...>(threads, runnable_objects...);  // line A
}
// Run simulation with visualization and callbacks. Blocking function.
template <class... RunnableObjects>
void run_simulation(SimulatorPtr sim, VisualizerPtr visualizer, const std::size_t callback_threads = 1,
                    RunnableObjects&... runnable_objects)
{
  simulator_t& sim_ref{ *sim };
  // std::thread step_thread(&simulator_t::run, &(*sim));  // Mj sim
  std::vector<std::thread> threads{};

  run_thread(threads, sim_ref, runnable_objects...);

  ros::AsyncSpinner spinner(callback_threads);  // 1 thread for the controller

  // Run threads: Mj sim is already running at this point
  spinner.start();
  if (visualizer)
  {
    // Visualizer has to run on the main thread
    visualizer->run();  // Blocking
  }
  // sim->run();
  // Join the non-visual threads
  for (auto& thread : threads)
  {
    thread.join();
  }
  spinner.stop();
}
}  // namespace mj_ros

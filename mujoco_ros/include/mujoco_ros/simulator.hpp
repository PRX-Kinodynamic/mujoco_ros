#pragma once
#include "defs.h"
#include "mujoco/mujoco.h"
#include <sstream>

class mujoco_simulator_t : public std::enable_shared_from_this<mujoco_simulator_t>
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

public:
  mjModel* m;
  mjData* d;

  mujoco_simulator_t(const std::string& model_path, bool _save_trajectory, bool _visualize)
  {
    m = mj_loadXML(model_path.c_str(), NULL, NULL, 0);
    if (!m)
      std::cerr << "Error in loading model." << std::endl;

    d = mj_makeData(m);
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

    visualize = _visualize;
    if (visualize)
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
      mjv_makeScene(m, &scn, 1000);
      mjr_makeContext(m, &con, mjFONTSCALE_150);
      viewport = { 0, 0, 1200, 900 };

      glfwSetWindowUserPointer(window, this);
      glfwSetCursorPosCallback(window, [](GLFWwindow* window, double xpos, double ypos) {
        auto sim = static_cast<mujoco_simulator_t*>(glfwGetWindowUserPointer(window));
        sim->mouse_move(window, xpos, ypos);
      });
      glfwSetMouseButtonCallback(window, [](GLFWwindow* window, int button, int action, int mods) {
        auto sim = static_cast<mujoco_simulator_t*>(glfwGetWindowUserPointer(window));
        sim->mouse_button(window, button, action, mods);
      });
      glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
        auto sim = static_cast<mujoco_simulator_t*>(glfwGetWindowUserPointer(window));
        sim->scroll(window, xoffset, yoffset);
      });
    }
  }

  ~mujoco_simulator_t()
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

  void step_simulation()
  {
    for (int i = 0; i < m->nv; i++)
    {
      d->qacc_warmstart[i] = 0;
    }
    mj_step(m, d);
    if (visualize)
    {
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
      mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
      mjr_render(viewport, &scn, &con);
      glfwSwapBuffers(window);
      glfwPollEvents();
    }
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

  void reset_simulation()
  {
    mj_resetData(m, d);
    if (save_trajectory)
    {
      trajectory.clear();
      add_current_state_to_trajectory();
    }
  }

protected:
  mjrRect viewport;
  bool button_left, button_right, button_middle;
  double lastx, lasty;

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
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
  }
  void scroll(GLFWwindow* window, double xoffset, double yoffset)
  {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
  }
};

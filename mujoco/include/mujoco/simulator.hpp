#pragma once
#include "defs.h"
#include "mujoco/mujoco.h"

class mujoco_simulator_t : public std::enable_shared_from_this<mujoco_simulator_t>
{
  private:
  bool save_trajectory;
  std::vector<std::vector<double>> trajectory;
  std::vector<double> current_state;
  public:
  mjModel* m;
  mjData* d;

  mujoco_simulator_t(const std::string& model_path, bool _save_trajectory)
  {
    m = mj_loadXML(model_path.c_str(), NULL, NULL, 0);
    if(!m)
      std::cerr << "Error in loading model." << std::endl;
    
    d = mj_makeData(m);
    for (int i = 0; i < 1.0/m->opt.timestep; i++)
    {
      mj_step(m, d);
    }
    for (int i = 0; i < m->nq; i++)
    {
      std::cout << d->qpos[i] << " ";
    }
    std::cout << std::endl;
    save_trajectory = _save_trajectory;
  }

  ~mujoco_simulator_t()
  {
    mj_deleteData(d);
    mj_deleteModel(m);
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
};

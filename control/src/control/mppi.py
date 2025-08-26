import numpy as np
import torch
import rospy

from ml4kp_bridge.msg import Trajectory, Plan, PlanStep, SpacePoint
from visualization_msgs.msg import Marker


class mppi:

    def __init__(self, plant, horizon=100, sample_rollouts=100,  device = 'cuda'):
        self.dtype = torch.float32                                                            
        if device == 'cuda' and torch.cuda.is_available():
            self.device = 'cuda'
        else:
            self.device = 'cpu'

        self.plant = plant
        self.Xdim = self.plant.Xdim
        self.Udim = self.plant.Udim

        #------MPPI variables and constants----
        #Parameters
        self.horizon = horizon                      # Length of rollout horizon
        # self.sample_rollouts = 1                     # Number of sample rollouts
        self.sample_rollouts = sample_rollouts                     # Number of sample rollouts
        # self.sample_rollouts = 100                     # Number of sample rollouts
        # self.sample_rollouts = 800                     # Number of sample rollouts
        # self.dt = 1
        self.lambda_ = 0.1               # Temperature
        self.sigma = torch.Tensor([0.2, 0.5]).type(torch.float32).expand(self.horizon, self.sample_rollouts, 2).to(self.device)  # (T, K, 2)
        self.inv_sigma = 1.0 / self.sigma[0, 0, :]
        self.u_noise = torch.Tensor(self.horizon, self.sample_rollouts, self.Udim).type(self.dtype).to(self.device)                              # (T,K,2)

        self.ctrl = torch.zeros((self.horizon, self.Udim)).to(self.device) # Initial speed = 5.0 m/s
        self.state = torch.Tensor(self.sample_rollouts, self.horizon, self.Xdim).type(self.dtype)                               # (K,T,6)
        self.state_dot = torch.Tensor(self.sample_rollouts, self.horizon, self.Xdim).type(self.dtype)                               # (K,T,6)
        self.ctrl_cost = torch.Tensor(self.sample_rollouts, 2).type(self.dtype).to(self.device)                                   # (K,2)

        self.ctrl_change = torch.Tensor(self.horizon, self.Udim).type(self.dtype).to(self.device)                                  # (T,2)

        self.running_cost = torch.zeros(self.sample_rollouts).type(self.dtype).to(self.device)                                    # (K, )

        self.goal_ = torch.zeros(self.Xdim).to(self.device);

    @property    
    def goal(self):
        return self.goal_

    @goal.setter
    def goal(self, new_goal):
        self.goal_ = new_goal
        self.goal_.to(self.device)

    # @property    
    # def horizon(self):
    #     return self.horizon

    # @horizon.setter
    # def horizon(self, new_horizon):
    #     self.horizon = new_horizon

    # @property    
    # def sample_rollouts(self):
    #     return self.sample_rollouts

    # @sample_rollouts.setter
    # def sample_rollouts(self, new_sample_rollouts):
    #     self.sample_rollouts = new_sample_rollouts

    def control_cost(self, ctrl, noise):

        #  ctrl * lambda * inv_sigma * noise * 0.5
        self.ctrl_cost.copy_(ctrl).mul_(self.lambda_).mul_(self.inv_sigma).mul_(noise).mul_(0.5)
        running_cost_temp = self.ctrl_cost.abs_().sum(dim=1)
        # self.running_cost.copy_(running_cost_temp)
        return running_cost_temp
        
    # cost to go
    def cost(self, state, goal, ctrl, noise, t):

        state_cost = self.plant.cost(state, goal, t)

        ctrl_cost = self.control_cost(ctrl, noise)

        # map_o_height, map_o_width = self.obstacle_map.size()

        # # Use vehicle's actual position for traversability assessment
        # p_x = (map_o_height // 2 - pose[:,1]).clone().detach().to(dtype=torch.long, device=self.device)
        # p_y = (map_o_width // 2 + pose[:,0]).clone().detach().to(dtype=torch.long, device=self.device)
        
        # p_x[p_x<=0] = 0
        # p_x[p_x>=map_o_width] = map_o_width - 1
        # p_y[p_y<=0] = 0
        # p_y[p_y>=map_o_height] = map_o_height -1

        # obstacle_penalty = (self.obstacle_map[p_y, p_x]==255).float().to(self.running_cost.device)

        # self.running_cost.add_(euclidian_distance_squared*10).add_(obstacle_penalty*50)
        return state_cost + ctrl_cost

    def run(self, x0, u0):

        # t0 = time.time()
        dt = self.plant.dt

        Xdim = self.Xdim
        Udim = self.Udim

        self.running_cost.zero_()                                           # Zero running cost
        # state = x0.repeat(self.sample_rollouts, 1).cuda()                    # Repeat the init pose to sample size 
        # nn_input = u0.repeat(self.sample_rollouts, 1).to(self.device)                # Repeat the init input to sample size
        
        # Initialize storage for this rollout's trajectories
        current_rollout = torch.zeros(self.sample_rollouts, self.horizon, Xdim).to(self.device)

        # state = nn_input[:, :Xdim]                                             # Get the state from the input
        # cmd_u = nn_input[:, Xdim:Udim]                                          # Get the control from the input

        # elev_map = self.map_embedding                                              # Repeat the map embedding to sample size
        torch.normal(0, self.sigma, out=self.u_noise)                                # Generate noise based on the sigma
        
        # state[:,[0,1,2]] = state[:,[0,1,2]] * 0.1
        # state = self.util.scale_in(state, self.scale_state, 0)
        # map_offset = self.util.scale_in(map_offset, self.offset_scale, 1)
        state = torch.Tensor(self.sample_rollouts, self.horizon, self.Xdim).type(self.dtype).to(self.device)
        state_dot = torch.Tensor(self.sample_rollouts, self.horizon, self.Xdim).type(self.dtype).to(self.device)

        state[:,0,:] = x0.repeat(self.sample_rollouts, 1).to(self.device)
        # state_dot[:,0,:] = torch.zeros_like(x0), 1).to(self.device)
        # Loop the forward calculation till the horizon
        for t in range(1, self.horizon):
            cmd_u = (self.ctrl[t] + self.u_noise[t]).to(self.device) 

            cmd_u = self.plant.bound_control(cmd_u)
            # cmd_vel[:, 0].clamp_(self.min_vel, self.max_vel)                         # Clamp control velocity
            # cmd_vel[:, 1].clamp_(self.min_del, self.max_del)                         # Clamp control steering

            # Model query for next pose calculation
            with torch.no_grad():
                xdot = self.plant.xdot(state[:,t-1,:], cmd_u);
                # out_xy = self.util.ackermann_model(cmd_vel)
            # pose[Xdim:] = xdot
            # state[:,[0,1,5]] = out_xy[:,[0,1,5]]
            # state[:,0:Xdim] = xdot[:,0:Xdim]
            # model_output = state.detach().clone()
            
            # Scale the output to add it in pose
            # se2_pose = pose[:, [0,1,5]].clone()
            # model_out_scaled = self.util.scale_out(model_output.clone(), self.scale_state, 0)
            # pose_temp, state = self.util.get_next_batch_se2(model_output, se2_pose, self.scale_state)
            
            # pose[:, [0,1]] = pose_temp[:,[0,1]] 
            # pose[:, 2] += model_out_scaled[:, 2]
            # pose[:, 3:5] = model_out_scaled[:, 3:5] 
            # map_offset = self.util.get_next_offsets_se2(map_offset, se2_pose, pose_temp, self.offset_scale)

            # Add to self poses
            state[:, t, :] = self.plant.integrate(state[:,t-1,:], xdot)
            # state[:,t,:] = state[:,t-1,:] + xdot * dt
            # state_dot[:,t,:] = xdot

            # self.sampled_trajectories = self.state[:,t,:].cpu().numpy()

            # Calculate the cost for each pose
            # self.cost(pose, self.goal_tensor, self.ctrl[t], self.noise[t], t)
            self.running_cost += self.cost(state[:,t,:], self.goal_, self.ctrl[t], self.u_noise[t], t)
            
        # MPPI weighing
        self.running_cost -= torch.min(self.running_cost)
        self.running_cost /= -self.lambda_
        torch.exp(self.running_cost, out=self.running_cost)
        weights = self.running_cost / torch.sum(self.running_cost)+1e-6

        weights = weights.unsqueeze(1).expand(self.horizon, self.sample_rollouts, 2)
        weights_temp = weights.mul(self.u_noise)
        self.ctrl_change.copy_(weights_temp.sum(dim=1))
        self.ctrl += self.ctrl_change
        self.ctrl = self.plant.bound_control(self.ctrl)
        # self.ctrl[:,0].clamp_(self.min_vel, self.max_vel)
        # self.ctrl[:,1].clamp_(self.min_del, self.max_del)
        self.state = state.cpu();
        # return self.state

    def control_to_plan_step(self, ui):
        plan_step = PlanStep()
        duration = rospy.Duration(self.plant.dt)
        # plan_step.duration.
        plan_step.duration.data = rospy.Duration(self.plant.dt)
        for e in ui:
            plan_step.control.point.append(e.item())
        return plan_step

    def get_controls(self):
        # Trajectory, Plan
        plan = Plan()
        run_ctrl = self.ctrl.clone()
        for ui in run_ctrl:
            plan.steps.append(self.control_to_plan_step(ui));

        # shift all controls forward by 1, with last control replicated
        # self.ctrl = torch.roll(self.ctrl, shifts=-1, dims=0)
        # print(f"plan {plan}")
        return plan

    
    def compute_solution_trajectory(self, x0):
        dt = self.plant.dt
        traj = torch.zeros((1, self.horizon, self.Xdim)).to(self.device) 
        traj[0, 0, :] = x0
        ui = torch.zeros((1,self.plant.Udim)).to(self.device) 
        xi = torch.zeros((1,self.plant.Xdim)).to(self.device) 
        # print(f"ctrl {self.ctrl} ")
        for ti in range(1, self.horizon):
            with torch.no_grad():
                ui[:,] = self.ctrl[ti-1]
                xi[:,] = traj[0, ti-1,:]
                # print(f"ctrl {ui} ")
                xdot = self.plant.xdot(xi, ui);
                traj[0, ti, :] = xi + xdot * dt
        return traj;

    def get_solution_trajectory(self, x0):
        trajectories = self.compute_solution_trajectory(x0)
        traj_msg = Trajectory()

        # TODO: tot_trajs should always be one
        tot_trajs, horizon, Xdim = trajectories.shape
        
        # print(f"tot_trajs: {tot_trajs} horizon: {horizon} Xdim: {Xdim}")
        for traj_idx in range(tot_trajs):
            for t in range(horizon):
                traj_msg.data.append(SpacePoint())
                # print(f"trajectories[traj_idx,t] {trajectories[traj_idx,t]}")
                for e in trajectories[traj_idx,t]:
                    # print(f"traj_msg.data[-1].point {traj_msg.data[-1].point}")
                    traj_msg.data[-1].point.append(e.item())
                # marker.points.append(self.plant.to_point_marker(trajectories[traj_idx,t]))
        # print(f"traj_msg {traj_msg}")
        return traj_msg


    def solution_traj_to_marker(self, x0):
        traj = self.compute_solution_trajectory(x0)
        return self.trajectories_to_marker(traj, 1.0);


    # Put all or a ratio of the sampled trajs into a ros marker for visualization
    def sample_trajectories_to_marker(self, ratio = 1.0):
        return self.trajectories_to_marker(self.state, ratio)

    def trajectories_to_marker(self, trajectories, ratio = 1.0):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "world"
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.005

        # TODO: only publish total_trajs if != -1
        tot_trajs, horizon, Xdim = trajectories.shape
        
        trials = np.random.binomial(1, ratio, tot_trajs)

        # print(f"tot_trajs: {tot_trajs} horizon: {horizon} Xdim: {Xdim}")
        for traj_idx, pub in enumerate(trials):
            if not pub:
                continue
            for t in range(horizon):
                marker.points.append(self.plant.to_point_marker(trajectories[traj_idx,t]))
                if t > 0:  
                    marker.points.append(marker.points[-1])
            marker.points.pop()

        return marker

import numpy as np
import torch
from geometry_msgs.msg import Point

class Mushr(object):
    def __init__(self, device = 'cuda'):
        self.Xdim = 4
        self.Udim = 2
        self.wheelbase = 0.2965 
        self.dt = 0.1
        self.min_vel = 0.0
        self.max_vel = 1.0
        self.min_steer = -3.14159 / 2.0
        self.max_steer = +3.14159 / 2.0

        if device == 'cuda' and torch.cuda.is_available():
            self.device = 'cuda'
        else:
            self.device = 'cpu'

        
        self.vel_desired_idx = 1 ;
        self.steering_idx = 0 ;

        self.ubar_velocity_idx = 3;
        self.ubar_beta_idx = 1;

        self.accel_slope = 0.9898
        self.steering_param = 0.4203
        self.max_vel_param =  0.6228

        self.L = 0.2965
        self.sigmas = [0.75, 0.5]


    def bound_control(self, cmd_u):

        cmd_u[:, 0].clamp_(self.min_steer, self.max_steer)                         # Clamp control velocity
        cmd_u[:, 1].clamp_(self.min_vel, self.max_vel)                         # Clamp control velocity
        return cmd_u

    def integrate(self, x0, xdot0):
        x1 = torch.zeros_like(x0)
        x1[:,0:3] = x0[:,0:3] + xdot0[:,0:3] * self.dt
        x1[:,2] = torch.atan2(torch.sin(x1[:, 2]), torch.cos(x1[:, 2]))
        x1[:,3] = xdot0[:,3]
        return x1

    def ubar(self, xi, ui):
        # _state[0], &_state[1], &_state[2], &_ubar[0]
        v_current = xi[:, self.ubar_velocity_idx];
        steering = ui[:, self.steering_idx] ;
        v_desired = ui[:, self.vel_desired_idx] ;

        dv = v_desired - v_current;
        v_next = v_current + dv * self.accel_slope;
        beta = torch.atan(0.5 * torch.tan(steering * self.steering_param));

        ubar_vel = self.max_vel_param * v_next;

        return beta, ubar_vel;

    def xdot(self, xi, ui):

        beta, vt = self.ubar(xi, ui)

        theta = xi[:,2] ;

        cTh = torch.cos(theta + beta) 
        sTh = torch.sin(theta + beta) 
        wt = 2.0 * vt * torch.sin(beta) / self.L

        xd = torch.stack((vt * cTh, vt * sTh, wt, vt), dim=1)

        return xd

    def cost(self, state, goal, t):

        diff = torch.zeros_like(state).to(state.get_device())
        diff = state - goal.to(state.get_device())
        # print(f"state {state.get_device()} {state.shape}")
        # print(f"goal {goal.get_device()} {goal.shape}")
        # print(f"diff {diff.get_device()} {diff.shape}")
        diff[:, 2] = torch.atan2( torch.sin(diff[:, 2]), torch.cos(diff[:, 2]))

        error = torch.norm(diff, p=2, dim=1)
        return error

    def to_point_marker(self, state):
        msg = Point()
        # print(f"state: {state}")
        msg.x = state[0].item()
        msg.y = state[1].item()
        msg.z = 0
        return msg;

    def configuration(self, state):
        return state[:,0:2]
        # diff[:, 2] = torch.atan2( torch.sin(diff[:, 2]), torch.cos(diff[:, 2]))

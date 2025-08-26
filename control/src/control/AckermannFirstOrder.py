import numpy as np
import torch
from geometry_msgs.msg import Point

class AckermannFirstOrder(object):
    def __init__(self, device = 'cuda'):
        self.Xdim = 3
        self.Udim = 2
        self.wheelbase = 0.2965 
        self.dt = 0.1
        self.min_vel = 0.0
        self.max_vel = 0.6
        self.min_steer = -0.75
        self.max_steer = +0.75

        if device == 'cuda' and torch.cuda.is_available():
            self.device = 'cuda'
        else:
            self.device = 'cpu'

        self.one_x = torch.zeros((2,2)) # [1]_x
        self.one_x[0,1] = -1.0
        self.one_x[1,0] = +1.0
        self.one_x = self.one_x.to(self.device)

    def bound_control(self, cmd_u):

        cmd_u[:, 0].clamp_(self.min_vel, self.max_vel)                         # Clamp control velocity
        cmd_u[:, 1].clamp_(self.min_steer, self.max_steer)                         # Clamp control velocity
        return cmd_u

    # def adjoint(self, xi):
    #     th = xi[:,2]
    #     Adj = torch.zeros((2,2))
    #     Adj[0,0] = torch.cos(th)
    #     Adj[0,1] = -torch.sin(th)
    #     Adj[1,0] = torch.sin(th)
    #     Adj[1,1] = torch.cos(th)
    #     Adj[:2,2] = self.one_x * xi[:,:2]
    #     Adj[2,2] = 1.0
    #     return Adj.to(self.device)

    def integrate(self, x0, xdot0):
        x1 = x0 + xdot0 * self.dt
        x1[:,2] = torch.atan2(torch.sin(x1[:, 2]), torch.cos(x1[:, 2]))
        return x1

    def xdot(self, xi, ui):

        theta = xi[:,2]

        velocity = ui[:, 0]
        steering = ui[:, 1]
        # dt = 0.1

        # Calculate the change in orientation (dtheta)
        dtheta = velocity * torch.tan(steering) / self.wheelbase

        # Calculate change in x and y coordinates
        dx = velocity * torch.cos(theta)
        dy = velocity * torch.sin(theta)

        pose_change = torch.stack((dx, dy, dtheta), dim=1)

        return pose_change

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


import numpy as np
import torch
from geometry_msgs.msg import Point

class AckermannFirstOrder(object):
    def __init__(self):
        self.Xdim = 3
        self.Udim = 2
        self.wheelbase = 0.2965 
        self.dt = 0.1
        self.min_vel = 0.0
        self.max_vel = 0.6
        self.min_steer = -0.6
        self.max_steer = +0.6

    def bound_control(self, cmd_u):

        cmd_u[:, 0].clamp_(self.min_vel, self.max_vel)                         # Clamp control velocity
        cmd_u[:, 1].clamp_(self.min_steer, self.max_steer)                         # Clamp control velocity
        return cmd_u


    def xdot(self, u0):

        velocity = u0[:, 0]
        steering = u0[:, 1]
        # dt = 0.1

        # Calculate the change in orientation (dtheta)
        dtheta = velocity / self.wheelbase * torch.tan(steering)

        # Calculate change in x and y coordinates
        dx = velocity * torch.cos(dtheta)
        dy = velocity * torch.sin(dtheta)

        # Stack the changes in x, y, and theta into one tensor
        pose_change = torch.stack((dx, dy, dtheta), dim=1)

        return pose_change

    def cost(self, state, goal, t):

        diff = np.zeros_like(state)
        diff[:2] = state[:2] - goal[:2]
        diff[2] = torch.atan2( torch.sin(state[2]), torch.cos(state[2]))

        error = torch.norm(diff, p=2, dim=1)
        return error

    def to_point_marker(self, state):
        msg = Point()
        msg.x = state[0]
        msg.y = state[1]
        msg.z = 0
        return msg;



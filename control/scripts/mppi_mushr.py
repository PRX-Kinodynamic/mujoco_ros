import numpy as np
import torch

from control.mppi import mppi
from control.Mushr import Mushr

import math
import rospy
from visualization_msgs.msg import Marker
from ml4kp_bridge.msg import SpacePoint
from control.srv import MPPI, MPPIResponse
from grid_map_msgs.msg import GridMap
from scipy.spatial.transform import Rotation as SciPyRot

class mppi_ackermann:

    def __init__(self):

        self.visualize = rospy.get_param("~visualize", True)
        self.horizon = rospy.get_param("~horizon", 100)
        self.sample_rollouts = rospy.get_param("~sample_rollouts", 10)
        # self.blue_endcaps_topic  = rospy.get_param("~blue_endcaps_topic", "")

        self.sampled_trajs_publisher = rospy.Publisher("/mppi/trajectories/sampled", Marker, queue_size=1, latch=True)
        self.predicted_traj_publisher = rospy.Publisher("/mppi/trajectories/predicted", Marker, queue_size=1, latch=True)
        self.state_subscriber = rospy.Subscriber("/mppi/x0", SpacePoint, self.x0_callback)

        self.service = rospy.Service('/mppi/run', MPPI, self.mppi_service)

        self.plant = Mushr()
        self.controller = mppi(self.plant, self.horizon, self.sample_rollouts)
        self.u0 = torch.zeros(self.plant.Udim)
        self.x0 = torch.zeros(self.plant.Xdim)
        self.goal = torch.zeros(self.plant.Xdim)
        self.controller.goal = torch.Tensor([10,10,0])
        self.controller.obstacle_distance = 1.0
        self.controller.obstacle_penalty = 1

        self.grid_subscriber = rospy.Subscriber("/environment/grid", GridMap, self.grid_callback)
        # self.controller.goal = torch.Tensor([10,10,0])
        print("MPPI ready")

    def grid_callback(self, msg):
        self.controller.grid_environment = msg



    def x0_callback(self, msg):
        self.x0[0] = msg.point[0]
        self.x0[1] = msg.point[1]
        self.x0[2] = msg.point[2]
        self.x0[3] = msg.point[3]

        self.controller.goal = torch.Tensor([10,10,0,0])

        self.controller.run(self.x0, self.u0);

        sampled_marker_msg = self.controller.sample_trajectories_to_marker(1.0)
        predicted_marker_msg = self.controller.solution_traj_to_marker(self.x0)

        self.sampled_trajs_publisher.publish(sampled_marker_msg);
        self.predicted_traj_publisher.publish(predicted_marker_msg);

    def mppi_service(self, req):
        start_time = rospy.Time.now()

        self.x0[0] = req.start.point[0]
        self.x0[1] = req.start.point[1]
        self.x0[2] = req.start.point[2]
        self.x0[3] = req.start.point[3]

        self.goal[0] = req.goal.point[0]
        self.goal[1] = req.goal.point[1]
        self.goal[2] = req.goal.point[2]
        self.goal[3] = req.goal.point[3]
        
        self.controller.goal = self.goal
        # self.controller.goal = torch.Tensor([10,10,0])

        self.controller.run(self.x0, self.u0);

        response = MPPIResponse()

        response.plan = self.controller.get_controls();
        response.trajectory = self.controller.get_solution_trajectory(self.x0);

        if self.visualize:
            sampled_marker_msg = self.controller.sample_trajectories_to_marker(1.0)
            predicted_marker_msg = self.controller.solution_traj_to_marker(self.x0)

            self.sampled_trajs_publisher.publish(sampled_marker_msg);
            self.predicted_traj_publisher.publish(predicted_marker_msg);

        end_time = rospy.Time.now()
        response.compute_time.data = end_time - start_time 


        return response;



if __name__ == '__main__':
    rospy.init_node("MppiAckermann")
    node = mppi_ackermann()
    rospy.spin()



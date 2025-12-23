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
from prx_models.msg import MushrObservation, MushrControl
from interface.node_status import NodeStatusHelper
from interface.msg import NodeStatus
import tf

class mppi_mushr:

    def __init__(self):

        self.node_status = NodeStatusHelper("/nodes/mppi")

        self.visualize = rospy.get_param("~visualize", True)
        self.horizon = rospy.get_param("~horizon", 100)
        self.sample_rollouts = rospy.get_param("~sample_rollouts", 10)
        self.pose_topicname = rospy.get_param("~pose_topicname", "")
        self.frequency = rospy.get_param("~frequency", "")
        self.viz_sampling = rospy.get_param("~viz_sampling", 1.0)
        self.control_topicname = rospy.get_param("~control_topicname", "")
        seed = rospy.get_param("~seed", 231192)


        self.plant = Mushr()
        self.controller = mppi(self.plant, self.horizon, self.sample_rollouts, seed=int(seed))
        self.u0 = torch.zeros(self.plant.Udim)
        self.x0 = torch.zeros(self.plant.Xdim)
        self.goal = torch.zeros(self.plant.Xdim)
        self.controller.goal = torch.Tensor([1,5,0,0])
        self.controller.obstacle_distance = 1.0
        self.controller.obstacle_penalty = 2

        self.sampled_trajs_publisher = rospy.Publisher("/mppi/trajectories/sampled", Marker, queue_size=1, latch=True)
        self.predicted_traj_publisher = rospy.Publisher("/mppi/trajectories/predicted", Marker, queue_size=1, latch=True)
        self.control_publisher = rospy.Publisher(self.control_topicname, MushrControl, queue_size=1, latch=True)
        
        self.state_subscriber = rospy.Subscriber("/mppi/x0", SpacePoint, self.x0_callback)
        self.pose_subscriber = rospy.Subscriber(self.pose_topicname, MushrObservation, self.pose_callback)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frequency), self.run)

        self.service = rospy.Service('/mppi/run', MPPI, self.mppi_service)


        self.grid_subscriber = rospy.Subscriber("/environment/grid", GridMap, self.grid_callback)
        # self.controller.goal = torch.Tensor([10,10,0])
        self.previous_pose = torch.zeros(self.plant.Xdim)

        self.ctrl_msg = MushrControl()
        self.prev_time = rospy.Time.now()

        self.node_status.status = NodeStatus.READY

        print("MPPI ready")

    def grid_callback(self, msg):
        self.controller.grid_environment = msg

    def pose_callback(self, msg):
        self.previous_pose = self.x0;

        self.x0[0] = msg.pose.position.x
        self.x0[1] = msg.pose.position.y
        qw =  msg.pose.orientation.w;
        qx =  msg.pose.orientation.x;
        qy =  msg.pose.orientation.y;
        qz =  msg.pose.orientation.z;
        _,_,theta = tf.transformations.euler_from_quaternion([qx,qy,qz,qw])
        self.x0[2] = theta

        ti = rospy.Time.now()
        dt = (ti - self.prev_time).to_sec()
        self.prev_time = ti
        # self.x0[3] = torch.norm((self.previous_pose - self.x0)[0:2], p=2, dim=0)/dt
        self.x0[3] = 0

    def run(self, event):
        if self.node_status.status == NodeStatus.RESTART:
            exit(0)
        if self.node_status.status != NodeStatus.RUNNING:
            return
            
        start_time = rospy.Time.now()

        self.controller.run(self.x0, self.u0);

        # plan = self.controller.get_controls();
        # trajectory = self.controller.get_solution_trajectory(self.x0);

        if self.visualize:
            sampled_marker_msg = self.controller.sample_trajectories_to_marker(self.viz_sampling)
            predicted_marker_msg = self.controller.solution_traj_to_marker(self.x0)

            self.sampled_trajs_publisher.publish(sampled_marker_msg);
            self.predicted_traj_publisher.publish(predicted_marker_msg);

        self.ctrl_msg.steering_angle = self.controller.ctrl[0, self.plant.steering_idx]
        self.ctrl_msg.velocity = self.controller.ctrl[0, self.plant.vel_desired_idx]

        # shift all controls forward by 1, with last control replicated
        self.controller.ctrl = torch.roll(self.controller.ctrl, shifts=-1, dims=0)
        
        self.control_publisher.publish(self.ctrl_msg);
        end_time = rospy.Time.now()
        compute_time = end_time - start_time 
        # if self.visualize:
        #     print(f"Real dt: {compute_time.to_sec()} ")
        # end_time = rospy.Time.now()
        # response.compute_time.data = end_time - start_time 

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
    node = mppi_mushr()
    rospy.spin()



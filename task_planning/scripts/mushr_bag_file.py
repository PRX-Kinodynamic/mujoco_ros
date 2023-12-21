#!/usr/bin/env python
import rosbag
import rospkg
import numpy as np
# import bagpy
# from bagpy import bagreader

base_path = rospkg.RosPack().get_path('task_planning') + '/data/'
namespace = 'mushr'
rosbag_file = base_path + namespace + ".bag"

ml4kp_plan_file = base_path + namespace + "_plan.txt"
simulation_trajectory_file = base_path + namespace + "_trajectory.txt"

# b = bagreader(rosbag_file)
# plan_message = b.message_by_topic('/' + namespace + '/plan')
# pose_message = b.message_by_topic('/' + namespace + '/pose')

bag = rosbag.Bag(rosbag_file)

topics = ['/' + namespace + '/plan']
controls = []
durations = []
for topic, msg, t in bag.read_messages(topics=topics):
    for control in msg.controls:
        controls.append([control.steering_angle.data, control.velocity.data])
    for duration in msg.durations:
        durations.append(duration.data.to_sec())
plans = np.hstack([np.array(controls), np.array(durations)[:, np.newaxis]])
np.savetxt(ml4kp_plan_file, plans, delimiter=',')

topics = ['/' + namespace + '/pose']
poses = []
for topic, msg, t in bag.read_messages(topics=topics):
    current_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
    poses.append(current_pose)
poses = np.array(poses)
np.savetxt(simulation_trajectory_file, poses, delimiter=',')

bag.close()
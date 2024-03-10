#!/usr/bin/env python
import rospy
import roslaunch
import subprocess
import numpy as np
import tqdm
from std_msgs.msg import Empty
from motion_planning.msg import PlanningResult

reset_topic = "/mushr/reset"
planning_result_topic = "/mushr/planning_result"


def convert_to_string(goal_config):
    return str(goal_config[0]) + ', ' + str(goal_config[1]) + ', ' + str(goal_config[2])


class ReplanningExperiment:
    def __init__(self, use_rogue=False):
        rospy.init_node('replanning_experiment', anonymous=True)
        self.reset_pub = rospy.Publisher(reset_topic, Empty, queue_size=1, latch=True)
        self.planning_result_sub = rospy.Subscriber(planning_result_topic, PlanningResult,
                                                    self.planning_result_callback)

        self.package = 'task_planning'

        run_types = [
            'single_shot_stanley',
            'replanning_open_loop',
            'replanning_stanley',
        ]

        self.run_type = run_types[2]

        if 'replanning' in self.run_type:
            self.launch_file = 'replanning.launch'
        else:
            self.launch_file = 'single_shot.launch'

        self.successes = []
        self.times = []
        self.collisions = []
        self.timeouts = []

        self.args = ['visualize:=false', 'record:=false', 'use_rogue:=' + str(use_rogue)]

    def planning_result_callback(self, msg):
        if msg.goal_reached.data:
            self.successes.append(1)
            self.times.append(msg.total_time.data)
        elif msg.in_collision.data:
            self.collisions.append(1)
        else:
            self.timeouts.append(1)

    def run(self, goal_config, num_trials=2, planning_cycle_duration=1.0):
        # max_cycles = int(30.0 / planning_cycle_duration)
        for i in tqdm.tqdm(range(num_trials)):
            # Send a reset message and wait
            rospy.loginfo("Sending reset message")
            self.reset_pub.publish(Empty())
            rospy.sleep(1.0)

            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments([self.package, self.launch_file])[0]
            launch_args = self.args + [
                'goal_config:=' + convert_to_string(goal_config),
                'planning_cycle_duration:=' + str(planning_cycle_duration),
                'seed:=' + str(i),
                'max_cycles:=40',
                'propagate_dynamics:=' + 'true' if 'stanley' in self.run_type else 'false',
            ]

            parent = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"), [(roslaunch_file, launch_args)])
            parent.start()

            try:
                parent.spin()
            finally:
                parent.shutdown()

            rospy.sleep(0.5)

        rospy.loginfo("Successes: " + str(len(self.successes)) + " out of " + str(num_trials))
        rospy.loginfo("Collisions: " + str(len(self.collisions)) + " out of " + str(num_trials))
        rospy.loginfo("Timeouts: " + str(len(self.timeouts)) + " out of " + str(num_trials))
        rospy.loginfo("Average time: " + str(np.mean(self.times)))


if __name__ == '__main__':
    try:
        replanning_experiment = ReplanningExperiment(use_rogue=False)
        replanning_experiment.run(np.array([0.5, 5.0, 0.0]), 10, 1)
    except rospy.ROSInterruptException:
        pass

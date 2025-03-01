#!/usr/bin/env python
import time
import rospy
import roslaunch
import subprocess
import numpy as np
from std_msgs.msg import Empty
from motion_planning.msg import PlanningResult

reset_topic = "/mushr/reset"
planning_result_topic = "/mushr/planning_result"


RUN_PARAMS = [
    {
        "run": 0,
        "check": {"safe_min": 0.05, "safe_mul": 0.25, "safe_mul_quad": 0.0},
        "planning": {"safe_min": 0.05, "safe_mul": 0.25, "safe_mul_quad": 0.0},
    },
    # Traditional Logic (Planning less conservative than Safety Check)
    # {
    #     "run": 1,
    #     "check": {"safe_min": 0.03, "safe_mul": 0.17, "safe_mul_quad": 0.02},
    #     "planning": {"safe_min": 0.02, "safe_mul": 0.15, "safe_mul_quad": 0.01},
    # },
    # {
    #     "run": 2,
    #     "check": {"safe_min": 0.03, "safe_mul": 0.18, "safe_mul_quad": 0.02},
    #     "planning": {"safe_min": 0.02, "safe_mul": 0.16, "safe_mul_quad": 0.01},
    # },
    # {
    #     "run": 3,
    #     "check": {"safe_min": 0.04, "safe_mul": 0.18, "safe_mul_quad": 0.02},
    #     "planning": {"safe_min": 0.03, "safe_mul": 0.16, "safe_mul_quad": 0.01},
    # },
    # # Equal Parameters (Planning = Safety Check)
    # {
    #     "run": 4,
    #     "check": {"safe_min": 0.02, "safe_mul": 0.16, "safe_mul_quad": 0.02},
    #     "planning": {"safe_min": 0.02, "safe_mul": 0.16, "safe_mul_quad": 0.02},
    # },
    # {
    #     "run": 5,
    #     "check": {"safe_min": 0.03, "safe_mul": 0.17, "safe_mul_quad": 0.02},
    #     "planning": {"safe_min": 0.03, "safe_mul": 0.17, "safe_mul_quad": 0.02},
    # },
    # {
    #     "run": 6,
    #     "check": {"safe_min": 0.03, "safe_mul": 0.18, "safe_mul_quad": 0.02},
    #     "planning": {"safe_min": 0.03, "safe_mul": 0.18, "safe_mul_quad": 0.02},
    # },
    # # Reversed Logic (Planning more conservative than Safety Check)
    # {
    #     "run": 7,
    #     "check": {"safe_min": 0.02, "safe_mul": 0.16, "safe_mul_quad": 0.01},
    #     "planning": {"safe_min": 0.03, "safe_mul": 0.17, "safe_mul_quad": 0.02},
    # },
    # {
    #     "run": 8,
    #     "check": {"safe_min": 0.02, "safe_mul": 0.17, "safe_mul_quad": 0.01},
    #     "planning": {"safe_min": 0.03, "safe_mul": 0.18, "safe_mul_quad": 0.02},
    # },
    # {
    #     "run": 9,
    #     "check": {"safe_min": 0.03, "safe_mul": 0.16, "safe_mul_quad": 0.02},
    #     "planning": {"safe_min": 0.04, "safe_mul": 0.17, "safe_mul_quad": 0.03},
    # },
    # {
    #     "run": 10,
    #     "check": {"safe_min": 0.03, "safe_mul": 0.17, "safe_mul_quad": 0.02},
    #     "planning": {"safe_min": 0.04, "safe_mul": 0.18, "safe_mul_quad": 0.03},
    # },
]


def convert_to_string(goal_config):
    return str(goal_config[0]) + ", " + str(goal_config[1]) + ", " + str(goal_config[2])


class ReplanningExperiment:
    def __init__(self, use_rogue=False):
        rospy.init_node("replanning_experiment", anonymous=True)
        self.reset_pub = rospy.Publisher(reset_topic, Empty, queue_size=1, latch=True)
        self.planning_result_sub = rospy.Subscriber(
            planning_result_topic, PlanningResult, self.planning_result_callback
        )

        self.package = "task_planning"
        self.launch_file = "replanning.launch"

        self.successes = []
        self.times = []
        self.collisions = []
        self.timeouts = []

        self.args = []
        self.runs = []
        self.num_trials = 0
        # self.args = ['record:=false', 'use_rogue:=' + str(use_rogue)]

    def planning_result_callback(self, msg):
        if msg.goal_reached.data:
            self.successes.append(1)
            self.times.append(msg.total_time.data)
            self.runs.append([1, 0, 0, msg.total_time.data])
        elif msg.in_collision.data:
            self.collisions.append(1)
            self.runs.append([0, 1, 0, msg.total_time.data])
        else:
            self.timeouts.append(1)
            self.runs.append([0, 0, 1, msg.total_time.data])

    def run(
        self,
        goal_config,
        num_trials=2,
        planning_cycle_duration=1.0,
        use_contingency=False,
        max_cycles=90,
        preprocess_timeout=0.05,
        postprocess_timeout=0.05,
        file_name="runs",
    ):
        for i in range(num_trials):
            # Send a reset message and wait
            rospy.loginfo("Sending reset message")
            self.reset_pub.publish(Empty())
            rospy.sleep(1.0)

            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
                [self.package, self.launch_file]
            )[0]
            launch_args = self.args + [
                "max_cycles:=" + str(max_cycles),
                "goal_config:=" + convert_to_string(goal_config),
                "planning_cycle_duration:=" + str(planning_cycle_duration),
                "preprocess_timeout:=" + str(preprocess_timeout),
                "postprocess_timeout:=" + str(postprocess_timeout),
                "id:=" + str(i),
            ]
            if use_contingency:
                launch_args += ["use_contingency:=true"]

            parent = roslaunch.parent.ROSLaunchParent(
                rospy.get_param("/run_id"), [(roslaunch_file, launch_args)]
            )
            parent.start()

            try:
                parent.spin()
            finally:
                parent.shutdown()

            rospy.sleep(0.5)

            self.save_results(file_name)

            self.num_trials += 1

    def save_results(self, file_name):
        final_results = [
            len(self.successes),
            len(self.collisions),
            len(self.timeouts),
            np.mean(self.times),
        ]

        runs = self.runs.copy()

        runs.insert(
            0,
            final_results,
        )

        runs = np.array(runs)
        np.savetxt(
            "/Users/htnamus/All_Stuff/Programming_Stuff/ros_workspace/data/kraft/"
            + file_name
            + ".csv",
            runs,
            delimiter=",",
        )

        rospy.loginfo(
            "Successes: " + str(len(self.successes)) + " out of " + str(self.num_trials)
        )
        rospy.loginfo(
            "Collisions: "
            + str(len(self.collisions))
            + " out of "
            + str(self.num_trials)
        )
        rospy.loginfo(
            "Timeouts: " + str(len(self.timeouts)) + " out of " + str(self.num_trials)
        )
        rospy.loginfo("Average time: " + str(np.mean(self.times)))

        return final_results


def write_params(params):
    fpath = "/Users/htnamus/All_Stuff/Programming_Stuff/ML4KP-devel/resources/input_files/examples/replanning/planning_simulation.yaml"
    planning_params = params["planning"]
    check_params = params["check"]
    # Read existing yaml content
    with open(fpath, "r") as f:
        yaml_content = f.read()

    # Split into lines
    lines = yaml_content.split("\n")

    # Remove old safety params sections
    new_lines = []
    skip = False
    for line in lines:
        if line.startswith("plan_safety_params:") or line.startswith(
            "contingency_safety_params:"
        ):
            skip = True
            continue
        if skip and line.strip().startswith("safe_"):
            continue
        if skip and (not line.strip() or not line.startswith(" ")):
            skip = False
        if not skip:
            new_lines.append(line)

    # Add new safety params
    new_lines.append("plan_safety_params:")
    new_lines.append("  safe_min: {}".format(planning_params["safe_min"]))
    new_lines.append("  safe_mul: {}".format(planning_params["safe_mul"]))
    new_lines.append("  safe_quad_mul: {}".format(planning_params["safe_mul_quad"]))
    new_lines.append("")
    new_lines.append("contingency_safety_params:")
    new_lines.append("  safe_min: {}".format(check_params["safe_min"]))
    new_lines.append("  safe_mul: {}".format(check_params["safe_mul"]))
    new_lines.append("  safe_quad_mul: {}".format(check_params["safe_mul_quad"]))

    # Write back to file
    with open(fpath, "w") as f:
        f.write("\n".join(new_lines))


def run_experiment(params):
    received_keyboard_interrupt = False
    write_params(params)

    file_name = f"run_{params['run']}_plan_min{str(params['planning']['safe_min']).replace('.','p')}_mul{str(params['planning']['safe_mul']).replace('.','p')}_quad{str(params['planning']['safe_mul_quad']).replace('.','p')}_check_min{str(params['check']['safe_min']).replace('.','p')}_mul{str(params['check']['safe_mul']).replace('.','p')}_quad{str(params['check']['safe_mul_quad']).replace('.','p')}"

    # Start roscore
    try:
        roscore_process = subprocess.Popen(["roscore"])
        rospy.sleep(2.0)  # Wait for roscore to initialize
    except Exception as e:
        print("Error starting roscore:", e)
        return

    # Start rosrun rqt_image_view rqt_image_view
    try:
        rqt_process = subprocess.Popen(["rosrun", "rqt_image_view", "rqt_image_view"])
        rospy.sleep(1.0)  # Wait for rqt_image_view to initialize
    except Exception as e:
        print("Error starting rqt_image_view:", e)
        return

    # Start roslaunch mujoco_ros mushr.launch
    try:
        mushr_process = subprocess.Popen(["roslaunch", "mujoco_ros", "mushr.launch"])
        rospy.sleep(1.0)  # Wait for mushr.launch to initialize
    except Exception as e:
        print("Error starting mushr.launch:", e)
        return

    replanning_experiment = ReplanningExperiment(use_rogue=False)
    try:
        replanning_experiment.run(
            np.array([1.0, 5.0, 0.0]),
            num_trials=30,
            planning_cycle_duration=1.0,
            use_contingency=True,
            max_cycles=100,
            preprocess_timeout=0.05,
            postprocess_timeout=0.1,
            file_name=file_name,
        )
    except rospy.ROSInterruptException:
        print("ROSInterruptException")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        received_keyboard_interrupt = True
    results = replanning_experiment.save_results(file_name)

    # Stop roslaunch mujoco_ros mushr.launch
    try:
        mushr_process.terminate()
    except Exception as e:
        pass
    try:
        rqt_process.terminate()
    except Exception as e:
        pass
    try:
        roscore_process.terminate()
    except Exception as e:
        pass

    try:
        mushr_process.wait()
    except Exception as e:
        pass
    try:
        rqt_process.wait()
    except Exception as e:
        pass
    try:
        roscore_process.wait()
    except Exception as e:
        pass

    # Sleep for 2 seconds
    time.sleep(10.0)

    return results, received_keyboard_interrupt


if __name__ == "__main__":
    all_results = []
    for run in RUN_PARAMS:
        result, received_keyboard_interrupt = run_experiment(run)
        all_results.append(result)
        if received_keyboard_interrupt:
            break

    best_success_run = np.argmax(np.array([result[0] for result in all_results]))
    lowest_collision_run = np.argmin(np.array([result[1] for result in all_results]))
    lowest_num_collisions = all_results[lowest_collision_run][1]
    lowest_timeout_run = np.argmin(np.array([result[2] for result in all_results]))
    lowest_timeout = all_results[lowest_timeout_run][2]
    best_success = all_results[best_success_run][0]
    best_success_time = all_results[best_success_run][3]

    print(f"Best success run: {best_success_run + 1}")
    print(f"Lowest collision run: {lowest_collision_run + 1}")
    print(f"Lowest timeout run: {lowest_timeout_run + 1}")
    print(f"Best success: {best_success}")
    print(f"Lowest number of collisions: {lowest_num_collisions}")
    print(f"Lowest timeout: {lowest_timeout}")
    print(f"Best success time: {best_success_time}")

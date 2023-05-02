#!/usr/bin/env python

# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from sensor_msgs.msg import JointState

joints_dict = {}


moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

## Getting Basic Information
## ^^^^^^^^^^^^^^^^^^^^^^^^^
# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

def joint_states_callback(message):

    joint_commands = JointState()

    joint_commands.header = message.header

    for i, name in enumerate(message.name):

        # Storing arm joint names and positions
        joints_dict[name] = message.position[i]

        # if name == "joint_left":

        #     # Adding additional panda_finger_joint2 state info (extra joint used in isaac sim)
        #     # panda_finger_joint2 mirrors panda_finger_joint1
        #     joints_dict["joint_right"] = message.position[i]

    joint_commands.name = joints_dict.keys()
    joint_commands.position = joints_dict.values()

    # Publishing combined message containing all arm and finger joints
    pub.publish(joint_commands)

    return

def go_to_joint_states_callback_isaac(message):

    joint_commands = JointState()

    joint_commands.header = message.header

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -tau / 8
    joint_goal[2] = tau/4
    joint_goal[3] = -tau / 4
    joint_goal[4] = 0
    joint_goal[5] = tau / 6  # 1/6 of a turn
    # joint_goal[6] = 0.1
    # joint_goal[7] = 0.1

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    current_joints = move_group.get_current_joint_values()


    for i, name in enumerate(message.name):

        # Storing arm joint names and positions
        joints_dict[name] = joint_goal[i]

        # if name == "joint_left":

        #     # Adding additional panda_finger_joint2 state info (extra joint used in isaac sim)
        #     # panda_finger_joint2 mirrors panda_finger_joint1
        #     joints_dict["joint_right"] = message.position[i]

    joint_commands.name = joints_dict.keys()
    joint_commands.position = joints_dict.values()

    # Publishing combined message containing all arm and finger joints
    pub.publish(joint_commands)

    return all_close(joint_goal, current_joints, 0.01)


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

if __name__ == "__main__":

    rospy.init_node("kuka_combined_joints_publisher")
    pub = rospy.Publisher("/joint_command", JointState, queue_size=1)

    #rospy.Subscriber("/joint_command_desired", JointState, joint_states_callback, queue_size=1)
    rospy.Subscriber("/joint_command_desired", JointState, go_to_joint_states_callback_isaac, queue_size=1)

    rospy.spin()


    """
    header: 
        seq: 170
        stamp: 
            secs: 3010
            nsecs: 716823688
        frame_id: "base_link"
        name: 
        - joint_a1
        - joint_a2
        - joint_a3
        - joint_a4
        - joint_a5
        - joint_a6
        position: [0.686481519975468, -2.3677175964755364, 2.5781044455248914, 2.145352880856928, 1.9185556285919494, -5.372349182452595]
        velocity: []
        effort: []
    """

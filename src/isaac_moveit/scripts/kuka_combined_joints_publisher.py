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
from geometry_msgs.msg import Pose, Quaternion

from math import pi, tau, dist, fabs, cos

from moveit_commander.conversions import pose_to_list

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from tf.transformations import euler_from_quaternion, quaternion_from_euler



class kuka_combined_joints_publisher:


    def __init__(self):


        self.joints_dict = {}
        
        self.joint_request = JointState()
        self.pose_request = Pose()

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        # Default group name
        self.group_name = "KUKA3_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        #self.move_group.allow_replanning(True)
        

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


        rospy.init_node("kuka_combined_joints_publisher")

        self.pub = rospy.Publisher("/joint_command", JointState, queue_size=1)

        # Control from Rviz
        rospy.Subscriber("/joint_command_desired", JointState, self.joint_states_callback, queue_size=1)
        # Control each robot from Isaac (1st select group, then get joint states)
        rospy.Subscriber("/joint_move_group_isaac", String, self.select_move_group, queue_size=1)
        #rospy.Subscriber("/joint_command_isaac", JointState, self.go_to_joint_states_callback_isaac, queue_size=1)

    # Rviz Control
    def joint_states_callback(self, message):

        joint_commands = JointState()

        joint_commands.header = message.header

        for i, name in enumerate(message.name):

            # Storing arm joint names and positions
            self.joints_dict[name] = message.position[i]

            # if name == "joint_left":

            #     # Adding additional panda_finger_joint2 state info (extra joint used in isaac sim)
            #     # panda_finger_joint2 mirrors panda_finger_joint1
            #     joints_dict["joint_right"] = message.position[i]

        joint_commands.name = self.joints_dict.keys()
        joint_commands.position = self.joints_dict.values()

        # Publishing combined message containing all arm and finger joints
        self.pub.publish(joint_commands)

        return
      
    def select_move_group(self, message):

        self.group_name = message.data
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        rospy.loginfo("Selected move group: %s", self.group_name)

        rospy.Subscriber("/joint_command_isaac", JointState, self.go_to_joint_states_callback_isaac, queue_size=1)
        rospy.Subscriber("/pose_command_isaac", Pose, self.go_to_pose_callback_isaac, queue_size=1)
        rospy.Subscriber("/cartesian_path_command_isaac", Pose, self.go_to_cartesian_path_callback_isaac, queue_size=1)
        return


    def go_to_cartesian_path_callback_isaac(self, message):

        # Set target pose of the end-effector
        target_pose = Pose()

        target_pose.position.x = message.position.x
        target_pose.position.y = message.position.y
        target_pose.position.z = message.position.z
        q = quaternion_from_euler(0.0, 0.0, 0.0)  # roll, pitch, yaw
        target_pose.orientation = Quaternion(*q)

        # Set a list of waypoints for the Cartesian path
        waypoints = []
        waypoints.append(target_pose)

        # Set the start state to the current state
        self.move_group.set_start_state_to_current_state()

        # Compute the Cartesian path
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, # waypoint poses
                                                                  0.01,      # eef_step
                                                                  0.0)       # jump_threshold

        joint_values = self.move_group.get_current_joint_values()

        # Move the arm along the computed path
        self.move_group.go()

        # self.move_group.execute(plan)

        # Get the joint values from the computed plan
        # joint_values = plan.joint_trajectory.points[-1].positions

        # Publish the joint commands
        joint_commands = JointState()
        # joint_commands.header = message.header
        joint_commands.position = joint_values
        joint_commands.velocity = [0.0] * len(joint_values)
        joint_commands.effort = [0.0] * len(joint_values)
        joint_commands.name = self.move_group.get_active_joints() 

        # Publishing combined message containing all arm and finger joints
        self.pub.publish(joint_commands)

        return

    def go_to_joint_states_callback_isaac(self, message):

        # Get current joint positions
        joint_goal = self.move_group.get_current_joint_values()

        # Get requested joint positions
        joint_goal = message.position

        # Go to requested joint positions
        self.move_group.go(joint_goal, wait=True)
        # self.move_group.stop()

        for i, name in enumerate(message.name):

            # Storing arm joint names and positions
            self.joints_dict[name] = joint_goal[i]

        # Creating joint command message
        joint_commands = JointState()
        joint_commands.header = message.header
        joint_commands.name = self.joints_dict.keys()
        joint_commands.position = self.joints_dict.values()

        # Publishing combined message containing all arm and finger joints
        self.pub.publish(joint_commands)

        # Clearing joint dictionary
        #self.joints_dict = {}

        # Variable to test if joint positions are within tolerance
        current_joints = self.move_group.get_current_joint_values()

        return self.all_close(joint_goal, current_joints, 0.01)
    

    def go_to_pose_callback_isaac(self, message):
        # NOT WORKING YET
        # Get current joint positions
        joint_commands = JointState()
        joint_commands.name = self.move_group.get_active_joints() 

        target_pose = Pose()

        target_pose.position.x = message.position.x
        target_pose.position.y = message.position.y
        target_pose.position.z = message.position.z
        q = quaternion_from_euler(0.0, 0.0, 0.0)  # roll, pitch, yaw
        target_pose.orientation = Quaternion(*q)

        self.move_group.set_pose_target(target_pose, end_effector_link="kr3_2_link_6")
        self.move_group.go(wait=True)

        # self.move_group.stop()
        pose_goal = self.move_group.get_current_joint_values()


        for i, name in enumerate(joint_commands.name):

            # Storing arm joint names and positions
            self.joints_dict[name] = pose_goal[i]

            # if name == "joint_left":

            #     # Adding additional panda_finger_joint2 state info (extra joint used in isaac sim)
            #     # panda_finger_joint2 mirrors panda_finger_joint1
            #     joints_dict["joint_right"] = message.position[i]



        # joint_commands.name = self.joints_dict.keys()
        joint_commands.position = self.joints_dict.values()


        # Publishing combined message containing all arm and finger joints
        self.pub.publish(joint_commands)

        current_joints = self.move_group.get_current_joint_values()

        #self.joints_dict = {}

        return self.all_close(self.pose_request, current_joints, 0.01)


    def all_close(self, goal, actual, tolerance):
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
            return self.all_close(goal.pose, actual.pose, tolerance)

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
    kuka_combined_joints_publisher()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


    """ What I send:

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

    """ What I get:

    header: 
        seq: 45 <-- Several messages, not only one
        stamp: 
            secs: 315
            nsecs: 450016452
        frame_id: "world"

    name: 
    - kr3_1_joint_a1
    - kr3_1_joint_a2
    - kr3_1_joint_a3
    - kr3_1_joint_a4
    - kr3_1_joint_a5
    - kr3_1_joint_a6
    - kr3_1_schunk_joint_left
    - kr3_1_schunk_joint_right
    - kr3_2_joint_a1
    - kr3_2_joint_a2
    - kr3_2_joint_a3
    - kr3_2_joint_a4
    - kr3_2_joint_a5
    - kr3_2_joint_a6
    - kr3_2_schunk_joint_left
    - kr3_2_schunk_joint_right
    - kr3_3_joint_a1
    - kr3_3_joint_a2
    - kr3_3_joint_a3
    - kr3_3_joint_a4
    - kr3_3_joint_a5
    - kr3_3_joint_a6
    - kr3_4_joint_a1
    - kr3_4_joint_a2
    - kr3_4_joint_a3
    - kr3_4_joint_a4
    - kr3_4_joint_a5
    - kr3_4_joint_a6
    - kr4_5_joint_a1
    - kr4_5_joint_a2
    - kr4_5_joint_a3
    - kr4_5_joint_a4
    - kr4_5_joint_a5
    - kr4_5_joint_a6
    - kr4_5_schunk_joint_left
    - kr4_5_schunk_joint_right
    position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8726288771072731, -1.0000287031046116, -0.785433093176108, -1.5708897833434887, 8.03748164791614e-05, 1.0472336069120263, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocity: []
    effort: []
    """

    """
    position: 
        x: 0.6
        y: 0.49601638140176374
        z: 0.3
    orientation: 
        x: 0.0
        y: 0.0
        z: 0.707
        w: 0.707
    """

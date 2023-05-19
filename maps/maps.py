# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core import World
from omni.isaac.core.prims import GeometryPrim, XFormPrim, RigidPrim
import omni.kit.commands
from pxr import Sdf, Gf, UsdPhysics
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
import numpy as np
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.physics_context import PhysicsContext

import carb

# PMC Library Imports
from pmclib import system_commands as sys   # PMC System related commands
from pmclib import xbot_commands as bot     # PMC Mover related commands
from pmclib import pmc_types                # PMC API Types
import time
import random

from omni.isaac.core.utils import viewports, extensions
from omni.isaac.core.utils.prims import set_targets

import asyncio
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion
from math import tau

import sys

# Action graph imports
import omni.graph.core as og

import rosgraph

#########################################################################################################
class MAPs(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        # # Positions are relative to parents, so set them with reversed values
        # SCENE GEOMETRY
        # env (group) spacing:
        self._env_spacing = 2.0
        self.last_update_time = time.time()

        # Lab Setup:
        self._lab_setup_position = np.array([0.0, 0.0, 0.0])  # Gf.Vec3f(0.5, 0.0, 0.0)
        self._lab_setup_orientation = np.array([0, 0, 0, 1])
        self._lab_setup_scale = 1.0

        # Shuttles Grid:
        self._grid_position = np.array([1.2877, -1.0415, 0.0])
        shuttle_orientation = np.pi/2
        self._grid_orientation = np.array([np.cos(shuttle_orientation/2), 0, 0, np.sin(shuttle_orientation/2)]) #Rotates 90 degrees around z-axis

        # Shuttles:
        self._number_shuttles = 2
        #self._shuttle_position = np.array([1.2277, -0.9815, 1.07])
        self._shuttle_position = np.array([0.30, 0.90, 1.07]) #([0.06, 0.06, 1.07])
        self._platform_limits = np.array([0.0, 0.0, 0.832, 0.596]) # x_min, y_min, x_max, y_max
        self._target = np.array([0.8, 0.52])
        self._shuttle_scale = 0.01
        # self.xbot_ids = [1, 2, 3, 4, 5, 6, 7, 8]
        self.xbot_ids = [i for i in range(1, self._number_shuttles + 1)] 

        # Trays
        self._number_tray_vial = 1
        self._tray_vial_position = np.array([0.30, 0.90, 1.10]) #([0.06, 0.06, 1.10])
        # self._tray_vial_position = np.array([1.2277, -1.2, 1])
        self._tray_vial_scale = 0.0098

        self._number_tray_beaker = 0
        self._tray_beaker_position = np.array([0.18, 0.06, 1.090])
        # self._tray_beaker_position = np.array([1.42, -1.2, 1])
        self._tray_beaker_scale = 0.0099

        # Flyways:
        # DEFINE FLYWAYS MATRIX
        self.flyways_matrix = np.array([[1, 1, 1],
                                        [1, 1, 1],
                                        [1, 1, 1], 
                                        [1, 1, 1]])
            
        # Flyway offsets    
        self._flyway_position = np.array([1.165, -0.92398, 0.99302])
        self._flyway_orientation = np.array([0, 0, 0, 1])
        self._flyway_scale = 0.01

        # Grid for BFS Algorithm
        self.grid = []
        for row in self.flyways_matrix:
            grid_row = []
            for cell in row:
                grid_row.extend([cell]*2)  # each cell in flyways_matrix is 2 cells in grid
            for _ in range(2):  # each cell in flyways_matrix is 2 cells in grid
                self.grid.append(grid_row)

        # Kuka Multiple Arms:
        self._kuka_arms_position = np.array([0.0, 0.0, 1.0])  # Gf.Vec3f(0.5, 0.0, 0.0)
        self._kuka_arms_orientation = np.array(euler_angles_to_quat([0, 0, 0]))
        self._kuka_arms_scale = 1.0

        # Stirrer
        self._stirrer_position = np.array([0.10295, -0.69649, 0.0369])  # Gf.Vec3f(0.5, 0.0, 0.0)
        self._stirrer_orientation = np.array(euler_angles_to_quat([np.pi/2, 0, np.pi]))
        self._stirrer_scale = 1.0

        # USD asset paths:
        #self.asset_folder = "omniverse://localhost/Projects/MAPs-AAU/Assets/"
        self.asset_folder = "/home/andrei/P10-MAP/assets/"
        self.asset_paths = {
            #"kr3": self.asset_folder + "kr3r540/kr3r540_v3/kr3r540_v3.usd",
            #"kr3": self.asset_folder + "kr3r540/kr3r540_v4/kr3r540_v4.usd", # Schunk Kr3
            "kr3": self.asset_folder + "kr3r540_v4/kr3r540_v4g.usd", # Schunk Kr3
            "kr4": self.asset_folder + "kr4r600/kr4r600_v2.usd", 
            "kuka_multiple": self.asset_folder + "kuka_multiple_arms/kuka_multiple_arms_5.usd",
            "franka": "omniverse://localhost/NVIDIA/Assets/Isaac/2022.2.1/Isaac/Robots/Franka/franka_alt_fingers.usd",
            "flyway": self.asset_folder + "flyways/flyway_segment.usd",
            # "shuttle": self.asset_folder + "120x120x10/acopos_shuttle_120.usd", # Basic shuttle
            "shuttle": self.asset_folder + "120x120x10/shuttle_wh.usd",
            "tray_vial" : self.asset_folder + "Trays/Tray_vial_w.usd",
            "tray_flask" : self.asset_folder + "Trays/Tray_beaker_w.usd",
            "vial" : self.asset_folder + "vials/vial.usd",
            "stirrer" : self.asset_folder + "Magnetic_stirrer/Magnetic_stirrer.usd",
            #"lab_setup": self.asset_folder + "Lab_setup_v2.usd" # Lab Setup with robots
            #"lab_setup": self.asset_folder + "Lab_setup_v1.usd"  # Lab Setup without robots
            "lab_setup": self.asset_folder + "Lab_setup_v0.usd" # Lab Setup without robots or Acopos Matrix
        }

        # DEFINE STATIONS
        self.station_info = {
            1: {
                'position': np.array([0.2701, -1.16645, 0.99038]), # Station Position
                'orientation': np.array(euler_angles_to_quat([0, 0, np.pi/2])), # Station Orientation
                'scale': 1.0,                       # Station Scale
                'asset': self.asset_paths["kr3"],   # Station Asset
                'asset_name': "kr3"                 # Station Asset Name
            },
            2: {
                'position': np.array([0.829, -1.16645, 0.99038]),
                'orientation': np.array(euler_angles_to_quat([0, 0, np.pi/2])),
                'scale': 1.0,
                'asset': self.asset_paths["kr3"],
                'asset_name': "kr3"
            },
            3: {
                'position': np.array([0.271, -0.20714, 0.99038]),
                'orientation': np.array(euler_angles_to_quat([0, 0, -np.pi/2])),
                'scale': 1.0,
                'asset': self.asset_paths["kr3"],
                'asset_name': "kr3"
            },
            4: {
                'position': np.array([0.829, -0.20714, 0.99038]),
                'orientation': np.array(euler_angles_to_quat([0, 0, -np.pi/2])),
                'scale': 1.0,
                'asset': self.asset_paths["kr3"],
                'asset_name': "kr3"
            },
            5: {
                'position': np.array([1.34418, -0.20714, 0.99038]),
                'orientation': np.array(euler_angles_to_quat([0, 0, -np.pi/2])),
                'scale': 1.0,
                'asset': self.asset_paths["kr4"],
                'asset_name': "kr4"
            }
        }

        self.prim_dict = {} # Dictionary to store shuttle prim paths

        self.current_pos_dict = {} # Dictionary to store shuttle current positions

        self.planning_group = String() # ROS topic name for move group
        self.joint_state_request = JointState()
        self.pose_request = Pose()  
        self.cartesian_path_request = PoseArray()
        # self.joint_state_request.name = ["joint_a1", "joint_a2","joint_a3", "joint_a4", "joint_a5","joint_a6"]


        self.control_switch = 0 # 0: Sim, 1: PMC

        return

    # This function is called to setup the assets in the scene for the first time
    # Class variables should not be assigned here, since this function is not called
    # after a hot-reload, its only called to load the world starting from an EMPTY stage
    def setup_scene(self):

        # Check if ROS master is running
        if not rosgraph.is_master_online():
            carb.log_error("Please run roscore before executing this script")

        # A world is defined in the BaseSample, can be accessed everywhere EXCEPT __init__
        world = self.get_world()
        world = World.instance()

        # stage.SetDefaultPrim(world)
        world.scene.add_default_ground_plane() # adds a default ground plane to the scene

        # Add physics context
        physx_context = PhysicsContext()

        # Enable GPU dynamics
        physx_context.enable_gpu_dynamics(True)

        # Add Xform reference for the shuttles
        world.scene.add(XFormPrim(prim_path="/World/LabSetup", name=f"LabSetup"))

        # Add Xform reference for the shuttles
        world.scene.add(XFormPrim(prim_path="/World/LabSetup/Grid", name=f"Grid"))

        # Add Xform reference for the flyways
        for i in range(len(self.flyways_matrix)):
            for j in range(len(self.flyways_matrix[i])):
                if self.flyways_matrix[i][j] == 1:
                    add_reference_to_stage(usd_path=self.asset_paths["flyway"],
                                           prim_path="/World/LabSetup/Grid/flyway_{}{}".format((i+1),(j+1)))
                    world.scene.add(GeometryPrim(prim_path="/World/LabSetup/Grid/flyway_{}{}".format((i+1),(j+1)),
                                                 name="flyway_{}{}_ref_geom".format(i+1, j+1), collision=True))
                    
        # Add shuttles references
        for i in range(self._number_shuttles):
            add_reference_to_stage(usd_path=self.asset_paths["shuttle"], prim_path="/World/LabSetup/Grid/shuttle_{}".format(i+1))
            world.scene.add(RigidPrim(prim_path="/World/LabSetup/Grid/shuttle_{}".format(i+1),
                                         name="shuttle_{}_ref_geom".format(i+1),
                                         position= self._shuttle_position + np.array([0.12 *i, 0, 0]),
                                         scale = np.full((3,), self._shuttle_scale),
                                         mass = 0.30))
            
        # Add Trays
        for i in range(self._number_tray_vial):
            add_reference_to_stage(usd_path=self.asset_paths["tray_vial"], prim_path="/World/LabSetup/Grid/tray_vial_{}".format(i+1))
            world.scene.add(RigidPrim(prim_path="/World/LabSetup/Grid/tray_vial_{}".format(i+1),
                                         name="tray_vial_{}_ref_geom".format(i+1),
                                         position= self._tray_vial_position + np.array([0.12 *i, 0, 0]),
                                         scale = np.full((3,), self._tray_vial_scale),
                                         mass = 0.15))

        for i in range(self._number_tray_beaker):
            add_reference_to_stage(usd_path=self.asset_paths["tray_flask"], prim_path="/World/LabSetup/Grid/tray_beaker_{}".format(i+1))
            world.scene.add(RigidPrim(prim_path="/World/LabSetup/Grid/tray_beaker_{}".format(i+1),
                                         name="tray_beaker_{}_ref_geom".format(i+1),
                                         position= self._tray_beaker_position + np.array([0.12 *i, 0, 0]),
                                         scale = np.full((3,), self._tray_beaker_scale),
                                         mass = 0.15))
        
        # Add stirrer
        add_reference_to_stage(usd_path=self.asset_paths["stirrer"],
                                prim_path="/World/Kuka_Multiple_Arms/Stirrer")
        
        world.scene.add(RigidPrim(prim_path ="/World/Kuka_Multiple_Arms/Stirrer",
                                            name="magneticstirrer",
                                            position = self._stirrer_position,
                                            orientation = self._stirrer_orientation,
                                            mass = 1))
        # Add Robots references
        add_reference_to_stage(usd_path=self.asset_paths["kuka_multiple"],
                                prim_path="/World/Kuka_Multiple_Arms")

        
        world.scene.add(Articulation(prim_path ="/World/Kuka_Multiple_Arms",
                                            name="Kuka_Multiple_Arms",
                                            position = self._kuka_arms_position,
                                            orientation = self._kuka_arms_orientation))
        
        return

    # Here we assign the class's variables this function is called after load button is pressed
    # regardless starting from an empty stage or not this is called after setup_scene and after
    # one physics time step to propagate appropriate physics handles which are needed to retrieve
    # many physical properties of the different objects
    async def setup_post_load(self):

        # Load World and Assets --CHECK
        self._world = self.get_world()
        #self._world.scene.enable_bounding_boxes_computations()

        # Camera Initial Viewport
        viewports.set_camera_view(eye=np.array([3.3, -0.7, 2.2]), target=np.array([0.8, -0.7, 1.05]))

        # Add USD Assets
        await self._add_lab_setup()
        await self._add_shuttles_grid()
        for i in range(len(self.flyways_matrix)):
            for j in range(len(self.flyways_matrix[i])):
                if self.flyways_matrix[i][j] == 1:
                    await self._add_flyway(i, j)


        # Shuttles Prim Dictionary 
        stage = omni.usd.get_context().get_stage()
        for shuttle_number in range(self._number_shuttles):
            shuttle_path = "/World/LabSetup/Grid/shuttle_{}".format(shuttle_number + 1)
            prim = stage.GetPrimAtPath(shuttle_path)
            if prim:
                key_name = "prim_{}".format(shuttle_number + 1)
                self.prim_dict[key_name] = prim
            else:
                print("Error: shuttle prim not found at path {}".format(shuttle_path))

        # Create rospy node to publish requested joint positions
        rospy.init_node('isaac_joint_request_publisher')
        self.pub_group = rospy.Publisher('/joint_move_group_isaac', String, queue_size=10)
        self.pub_joints = rospy.Publisher('/joint_command_isaac', JointState, queue_size=10)
        self.pub_pose = rospy.Publisher('/pose_command_isaac', Pose, queue_size=10)
        self.pub_cartesian_path = rospy.Publisher('/cartesian_path_command_isaac', PoseArray, queue_size=10)

        # Creating a action graph with ROS component nodes
        try:
            og.Controller.edit(
                {"graph_path": "/World/Kuka_Multiple_Arms/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
                        ("SubscribeJointState", "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
                        ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                        ("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                        ("PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "PublishTF.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                        ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                        ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        # Setting the /Kuka target prim to Articulation Controller node
                        ("SubscribeJointState.inputs:topicName", "joint_command"),
                        ("ArticulationController.inputs:usePath", False),
                        ("ArticulationController.inputs:robotPath", ""),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Setting the /Kuka target prim to Publish JointState node
        set_targets(
            prim = stage.GetPrimAtPath("/World/Kuka_Multiple_Arms/ActionGraph/PublishJointState"),
            attribute="inputs:targetPrim",
            target_prim_paths=["/World/Kuka_Multiple_Arms"]
        )

        # Setting the /Kuka target prim to Articulation Controller node
        set_targets(
            prim = stage.GetPrimAtPath("/World/Kuka_Multiple_Arms/ActionGraph/ArticulationController"),
            attribute="inputs:targetPrim",
            target_prim_paths=["/World/Kuka_Multiple_Arms"]
        )

        # Setting the /Kuka target prim to Publish Transform Tree node
        set_targets(
            prim = stage.GetPrimAtPath("/World/Kuka_Multiple_Arms/ActionGraph/PublishTF"),
            attribute = "inputs:targetPrims",
            target_prim_paths=["/World/Kuka_Multiple_Arms"]
        )

        self.targets_x, self.targets_y = self.create_random_coordinates(self._number_shuttles)

        # Control Switch
        if self.control_switch == 0:
            # self._world.add_physics_callback("sim_step", callback_fn=self.sim_xbots_movement_2)

            # rospy.init_node('isaac_test', anonymous=True)
            # self.pub_joints = rospy.Publisher("/joint_command_desired", queue_size=1)
            #self.on_impulse_event()

            self._world.add_physics_callback("sim_step_impulse", callback_fn=self.on_impulse_event)
            # self._world.add_physics_callback("sim_step", self.sim_xbots_movement)


        elif self.control_switch == 1:
            self._connect_pmc()  # Connect to PMC
            self._world.add_physics_callback("sim_step", callback_fn=self.read_xbots_positions) #callback names have to be unique
            self._world.add_physics_callback("sim_step_move", callback_fn=self.send_xbots_positions)

        return
    

    # Add Lab Setup reference
    async def _add_lab_setup(self):
        self._lab_setup_ref_geom = self._world.scene.get_object(f"LabSetup")
        # self._lab_setup_ref_geom.set_local_scale(np.array([self._lab_setup_scale]))
        self._lab_setup_ref_geom.set_world_pose(position=self._lab_setup_position,
                                                orientation=self._lab_setup_orientation)
        self._lab_setup_ref_geom.set_default_state(position=self._lab_setup_position,
                                                orientation=self._lab_setup_orientation)
        # self._lab_setup_ref_geom.set_collision_approximation("none")
        #self._convexIncludeRel.AddTarget(self._table_ref_geom.prim_path)

    # Add flyways to the scene
    async def _add_flyway(self, x, y):
        self._flyway_ref_geom = self._world.scene.get_object(f"flyway_{x+1}{y+1}_ref_geom")
        self._flyway_ref_geom.set_local_scale(np.array([self._flyway_scale]))
        self._flyway_ref_geom.set_world_pose(position = self._flyway_position + (-0.24 * (x), +0.24 * (y), 0))
        self._flyway_ref_geom.set_default_state(position = self._flyway_position)
        self._flyway_ref_geom.set_collision_approximation("none")

    # Add xForm shuttles reference
    async def _add_shuttles_grid(self):
        self._shuttles_grid_ref_geom = self._world.scene.get_object(f"Grid")
        self._shuttles_grid_ref_geom.set_world_pose(position=self._grid_position,
                                                orientation=self._grid_orientation)
        self._shuttles_grid_ref_geom.set_default_state(position=self._grid_position,
                                                orientation=self._grid_orientation)

    ## Interface Functions:
    async def _on_sim_control_event_async(self):
        world = self.get_world()
        self.targets_x, self.targets_y = self.create_random_coordinates(self._number_shuttles)
        world.remove_physics_callback("sim_step")
        world.add_physics_callback("sim_step", self.sim_xbots_movement)
        await world.play_async()
        return
    
    async def _on_real_control_event_async(self):
        world = self.get_world()
        world.remove_physics_callback("sim_step")
        self._world.add_physics_callback("sim_step", callback_fn=self.read_xbots_positions) #callback names have to be unique
        self._world.add_physics_callback("sim_step_move", callback_fn=self.send_xbots_positions)
        await world.play_async()
        return


    ## CONTROL FUNCTIONS
    # Function to move selected robot to desired joints position   
    def move_to_joint_state(self, planning_group, joint_state_request):
        self.planning_group = planning_group
        self.joint_state_request.position = joint_state_request
        self.pub_group.publish(self.planning_group)
        self.pub_joints.publish(self.joint_state_request)
        return
    
    # Function to move selected robot to desired pose 
    def move_to_pose(self, planning_group, position, orientation):
        self.planning_group = planning_group
        quaternion = euler_angles_to_quat(orientation)  # reverse the order of the angles to be rxyz as in ROS
        self.pose_request = self.create_pose_msg(position, quaternion)
        self.pub_group.publish(self.planning_group)
        self.pub_pose.publish(self.pose_request)
        return   

    # Function to move selected robot to desired pose using cartesian path
    def move_along_cartesian_path(self, planning_group, waypoints):
        self.planning_group = planning_group
        self.cartesian_path_request.header.stamp = rospy.Time.now()
        self.cartesian_path_request.header.frame_id = 'world'  # or whatever frame_id you are using

        for waypoint in waypoints:
            position, orientation = waypoint
            quaternion = euler_angles_to_quat(orientation)  # convert euler to quaternion
            pose = self.create_pose_msg(position, quaternion)
            self.cartesian_path_request.poses.append(pose)

        print("Cartesian path request: ", self.cartesian_path_request)
        self.pub_group.publish(self.planning_group)
        self.pub_cartesian_path.publish(self.cartesian_path_request)

        return
    


    # Function to move selected selected shuttle to desired position
    def move_shuttle_to_target(self, xbot_id, target_x, target_y):
        # Check if the xbot_id exists in xbot_ids
        if xbot_id in self.xbot_ids:
            # Update the corresponding target_x and target_y values
            self.targets_x[xbot_id - 1] = target_x
            self.targets_y[xbot_id - 1] = target_y
        else:
            # If the xbot_id doesn't exist in xbot_ids, raise an exception
            raise ValueError(f"xbot_id {xbot_id} not found in xbot_ids")
        
    def gripper_control(self, planning_group, state):
        if state == "open":
            self.move_to_joint_state(planning_group, [0.0 , 0.0])
        elif state == "close":
            self.move_to_joint_state(planning_group, [-0.0047 , -0.0047])
        else:
            raise ValueError(f"state {state} not found in gripper_control")


    async def _on_start_experiment_event_async(self):

        #self._world.add_physics_callback("sim_step", self.sim_xbots_movement)

        # self.targets_x, self.targets_y = ([0.06, 0.42],[0.06, 0.18]) #([x1, x2],[y1, y2])

        # self.move_to_joint_state(planning_group="KUKA4_arm", 
        #                          joint_state_request=[random.uniform(-2, 2), 0.4, 0.3, 0, 0.707, 0.707])

        # self.move_to_joint_state(planning_group="KUKA2_arm", 
        #                          joint_state_request=[1.0905, 0.62695, 0.788, 0.05935, 1.18533, 1.06728])


        # position_xy = self.platform_pos_to_coordinates(2,7, moveit_offset = False)
        # self.move_shuttle_to_target(1, position_xy[0], position_xy[1]) 
        # print("position_xy: ", position_xy)

        # self.planning_group = 'kr3_1_arm'

        # orientation = [0.0, np.pi/2, 0.0] # Gripper pointing down
        
        # # orientation = [0.0, 0.707, 0.0, 0.707]
        # # print("orientation: ", orientation)

        # position_xy = self.platform_pos_to_coordinates(2,7, moveit_offset = True)

        # position = [position_xy[0], position_xy[1] , 0.275]
        # # print("position_offset: ", position)
        # # # position = [0.855, -0.140, 0.30]


        # self.move_to_pose(self.planning_group, position, orientation)

        # self.planning_group = 'kr3_1_hand'
        # self.gripper_control(self.planning_group, "close")


        self.planning_group = 'kr3_1_arm'

        position_xy = self.platform_pos_to_coordinates(2,7, moveit_offset = True)

        waypoints = [
            ((position_xy[0], position_xy[1], 0.45), (0.0, np.pi/2, 0.0)),
            ((position_xy[0], position_xy[1], 0.30), (0.0, np.pi/2, 0.0)),
            # ([position_xy[0], position_xy[1] + 0.5, 0.30], [0.0, np.pi/2, 0.0]),
            # ([position_xy[0], position_xy[1] - 0.5, 0.30], [0.0, np.pi/2, 0.0]),
        ]

        self.move_along_cartesian_path(self.planning_group, waypoints)


        # self.planning_group = 'KUKA4_arm'
        # position = [0.6, -0.6 , 0.18]
        # orientation = [0 , 0 , 0.707 , 0.707]

        # self.move_to_pose(self.planning_group, position, orientation)


        # self.pub_cartesian.publish(self.pose_request)

        ## TBD USE CARTESIAN PATH

        return

    def on_impulse_event(self, step_size):
        # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
        og.Controller.set(og.Controller.attribute("/World/Kuka_Multiple_Arms/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)


    async def setup_pre_reset(self):
        # world = self.get_world()
        # if world.physics_callback_exists("sim_step"):
        #     world.remove_physics_callback("sim_step")
        return

    async def setup_post_reset(self):
        # await self._world.play_async()
        return

    def world_cleanup(self):
        return


    # Move xbots in simulation (No collision detection)
    def sim_xbots_movement(self, step_size):

        max_speed = 0.3 # m/s
        max_accel = 10.0 # m/s^2
        move_increment = step_size * max_speed 

        reached_dest = [False] * self._number_shuttles  # Initialize list of reached destinations

        for shuttle_number in range(self._number_shuttles):
            prim = self.prim_dict["prim_{}".format(shuttle_number + 1)]

            current_pos = prim.GetAttribute('xformOp:translate').Get() 

            # print("current pos: ", current_pos)
            # print("target_x", self.targets_x[shuttle_number], "shuttle_number: ", shuttle_number + 1)
            if self.targets_x[shuttle_number] == current_pos[0] and self.targets_y[shuttle_number] == current_pos[1]:
                reached_dest[shuttle_number] = True     

            # if all(reached_dest):
            #     print("All shuttles have reached their destinations!")

                return 
                
            # Move shuttle up
            if (self.targets_y[shuttle_number]) > current_pos[1]:
                prim.GetAttribute('xformOp:translate').Set((current_pos) + (0.0, move_increment, 0.0))
                if (current_pos[1] + move_increment) > self.targets_y[shuttle_number]:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0], self.targets_y[shuttle_number], current_pos[2]))               
                continue
            # Move shuttle down
            elif (self.targets_y[shuttle_number]) < current_pos[1]:
                prim.GetAttribute('xformOp:translate').Set((current_pos) - (0.0, move_increment, 0.0))
                if (current_pos[1] - move_increment) < self.targets_y[shuttle_number]:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0], self.targets_y[shuttle_number], current_pos[2])) 
                continue

            # #current_pos = prim.GetAttribute('xformOp:translate').Get()

            # Move shuttle right
            if (self.targets_x[shuttle_number]) > current_pos[0]:
                prim.GetAttribute('xformOp:translate').Set((current_pos) + (move_increment, 0.0, 0.0))
                if (current_pos[0] + move_increment) > self.targets_x[shuttle_number]:
                    prim.GetAttribute('xformOp:translate').Set((self.targets_x[shuttle_number], current_pos[1], current_pos[2]))
                continue
            # Move shuttle left
            elif (self.targets_x[shuttle_number]) < current_pos[0]:
                prim.GetAttribute('xformOp:translate').Set((current_pos) - (move_increment, 0.0 , 0.0))
                if (current_pos[0] - move_increment) < self.targets_x[shuttle_number]:
                    prim.GetAttribute('xformOp:translate').Set((self.targets_x[shuttle_number], current_pos[1], current_pos[2]))
                continue


    # Move xbots in simulation (Checking other shuttles in the path)
    def sim_xbots_movement_collision(self, step_size):

        max_speed = 1.0 # m/s
        move_increment = step_size * max_speed 

        for xbot in range(self._number_shuttles):

            prim = self.prim_dict["prim_{}".format(xbot + 1)]

            current_pos = prim.GetAttribute('xformOp:translate').Get()

            x_pos_control = []
            y_pos_control = []

            # Collect all shuttles positions
            for shuttle_number in range(self._number_shuttles):
                prim_others = self.prim_dict["prim_{}".format(shuttle_number + 1)]
                shuttles_pos = prim_others.GetAttribute('xformOp:translate').Get()
                x_pos_control.append(shuttles_pos[0])
                y_pos_control.append(shuttles_pos[1])

            # Decide which direction to move in
            dx = self.targets_x[xbot] - current_pos[0]
            dy = self.targets_y[xbot] - current_pos[1]

            for shuttle in range(self._number_shuttles):
                continue_flag = False
                if xbot != shuttle and ((current_pos[1] > (y_pos_control[shuttle] - 0.0602 - move_increment) and current_pos[1] < (y_pos_control[shuttle] + 0.0602 + move_increment)) or (current_pos[0] > (x_pos_control[shuttle] - 0.0602 - move_increment) and current_pos[0] < (x_pos_control[shuttle] + 0.0602 + move_increment))):                
                    continue_flag = True
                
                if continue_flag:
                    continue

                # if moving right is safe
                if dx > 0:
                    prim.GetAttribute('xformOp:translate').Set((current_pos) + (move_increment, 0.0, 0.0))
                    if (current_pos[0] + move_increment) > self.targets_x[xbot]:
                        prim.GetAttribute('xformOp:translate').Set((self.targets_x[xbot], current_pos[1], current_pos[2]))
                    break
                # if moving left is safe
                elif dx < 0:
                    prim.GetAttribute('xformOp:translate').Set((current_pos) - (move_increment, 0.0 , 0.0))
                    if (current_pos[0] - move_increment) < self.targets_x[xbot]:
                        prim.GetAttribute('xformOp:translate').Set((self.targets_x[xbot], current_pos[1], current_pos[2]))
                    break
                # if moving up is safe
                if dy > 0:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0], current_pos[1] + move_increment, current_pos[2]))
                    if (current_pos[1] + move_increment) > self.targets_y[xbot]:
                        prim.GetAttribute('xformOp:translate').Set((current_pos[0], self.targets_y[xbot], current_pos[2]))
                    break
                # if moving down is safe
                elif dy < 0:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0], current_pos[1] - move_increment, current_pos[2]))
                    if (current_pos[1] - move_increment) < self.targets_y[xbot]:
                        prim.GetAttribute('xformOp:translate').Set((current_pos[0], self.targets_y[xbot], current_pos[2]))
                    break


    # Read shuttles position and orientation from physical setup
    def read_xbots_positions(self, step_size):

        # Read info for every shuttle
        xbot_list = bot.get_all_xbot_info(1)
        xbot_positions = [(xbot.x_pos, xbot.y_pos, xbot.z_pos,
                           xbot.rx_pos, xbot.ry_pos, xbot.rz_pos,
                           xbot.xbot_state) for xbot in xbot_list]

        # Set position and orientation of shuttles
        for shuttle_number in range(self._number_shuttles):
            prim = self.prim_dict["prim_{}".format(shuttle_number + 1)]

            # Set position of shuttle
            prim.GetAttribute('xformOp:translate').Set((xbot_positions[shuttle_number][0],
                                                        xbot_positions[shuttle_number][1] ,
                                                        xbot_positions[shuttle_number][2] + 1.06))
 
            # Transform orientation from euler angles to quaternion
            quat_prim = (euler_angles_to_quat([xbot_positions[shuttle_number][3],
                                               xbot_positions[shuttle_number][4],
                                               xbot_positions[shuttle_number][5]]))
            quat = Gf.Quatd(*quat_prim)

            # Set Orientation of shuttle
            prim.GetAttribute('xformOp:orient').Set(quat)


    def send_xbots_positions(self, step_size):

        # Only update the Xbots if at least 2 seconds have passed since the last update
        if time.time() - self.last_update_time >= 2:

            #print(bot.get_xbot_status(xbot_id=xid).xbot_state)

            xbot_list = bot.get_all_xbot_info(1)
            xbot_positions = [(xbot.x_pos, xbot.y_pos, xbot.z_pos,
                            xbot.rx_pos, xbot.ry_pos, xbot.rz_pos,
                            xbot.xbot_state) for xbot in xbot_list]

            # Don't send commands while the xbots are moving
            if all(xbot_state[6] == pmc_types.XbotState.XBOT_IDLE for xbot_state in xbot_positions): #xbot_state[6] --> xbot_state
                # Get random unique targets for each shuttle
                targets_x, targets_y = self.create_random_coordinates(self._number_shuttles)
                print("target_x ", targets_x)
                print("target_y ", targets_y)
                bot.auto_driving_motion_si(8, xbot_ids=self.xbot_ids, targets_x=targets_x, targets_y=targets_y)
            else:
                print("Xbots are moving")

            self.last_update_time = time.time()


        # if bot.get_xbot_status(xbot_id=xid).xbot_state is pmc_types.XbotState.XBOT_IDLE:
        #     #self.sample_motions(input_id=xid)
        #     bot.auto_driving_motion_si(8, xbot_ids=xbot_ids, targets_x=targets_x, targets_y=targets_y)
        # # Recover Disabled xbot
        # elif bot.get_xbot_status(xbot_id=xid).xbot_state is pmc_types.XbotState.XBOT_DISABLED:
        #     bot.recover_accident_xbot(xbot_id=xid)
        #     print("Recovering xbot: ", xid)


    def create_random_coordinates(self, num_shuttles):

        x_coords = []
        y_coords = []
        coords_dict = {}

        for i in range(num_shuttles):
            while True:
                x = random.randint(0, 5) * 0.12 + 0.06
                y = random.randint(0, 7) * 0.12 + 0.06
                if (x, y) not in coords_dict:
                    coords_dict[(x, y)] = 1
                    break

            x_coords.append(x)
            y_coords.append(y)

        return x_coords, y_coords


    def sample_motions(self, input_id):
        max_speed = 1.0
        max_accel = 10.0
        bot.linear_motion_si(xbot_id=input_id, target_x=0.18, target_y=0.06,
                            max_speed=max_speed, max_accel=max_accel)
        bot.linear_motion_si(xbot_id=input_id, target_x=0.18, target_y=0.90,
                            max_speed=max_speed, max_accel=max_accel)
        bot.linear_motion_si(xbot_id=input_id, target_x=0.66, target_y=0.90,
                            max_speed=max_speed, max_accel=max_accel)
        bot.linear_motion_si(xbot_id=input_id, target_x=0.66, target_y=0.06,
                            max_speed=max_speed, max_accel=max_accel)

        # bot.rotary_motion_timed_spin(xbot_id=input_id,rot_time=3, target_rz=3.14,
        #                              max_speed=3.0, max_accel=max_accel)

        # bot.linear_motion_si(xbot_id=input_id, target_x=0.60, target_y=0.36,
        #                     max_speed=max_speed, max_accel=max_accel)
        # bot.rotary_motion_timed_spin(xbot_id=input_id,rot_time=3, target_rz=3.14,
        #                              max_speed=3.0, max_accel=max_accel)


    def wait_for_xbot_done(xid):
        while bot.get_xbot_status(xbot_id=xid).xbot_state is not pmc_types.XbotState.XBOT_IDLE:
            time.sleep(0.5)

    def _connect_pmc(self):

        # Connect to PMC
        if not sys.auto_connect_to_pmc():
            sys.connect_to_pmc("192.168.10.100") #sys.auto_connect_to_pmc()

        print("Connected: ", sys.auto_connect_to_pmc())
        print("Status: ", sys.get_pmc_status())

        # Gain mastership
        if not sys.is_master():
            sys.gain_mastership()

        print("Master: ", sys.is_master())

        # Activate xBots
        bot.activate_xbots()


    def create_pose_msg(self, position, orientation, frame_id=''):
        # Create a new Pose object
        pose = Pose()

        # Given a position and orientation as lists and a frame id as string, return a PoseStamped message object.
        header = Header()
        header.frame_id = frame_id
        header.stamp = rospy.Time.now()

        # Set the position field of the Pose object
        position_obj = Point()
        position_obj.x = position[0]
        position_obj.y = position[1]
        position_obj.z = position[2]
        pose.position = position_obj

        # Set the orientation field of the Pose object (rxyz)
        orientation_obj = Quaternion()
        orientation_obj.x = orientation[1]
        orientation_obj.y = orientation[2]
        orientation_obj.z = orientation[3]
        orientation_obj.w = orientation[0]
        pose.orientation = orientation_obj

        pose_stamped = PoseStamped(header=header, pose=pose)

        return pose

    def platform_pos_to_coordinates(self, x_pos, y_pos, moveit_offset=False, robot_arm=None): 
        """
        Description: Converts the acopos platform positions to the platform coordinates.
        Origin of the matrix is at the bottom left corner (0,0)
        Limits of the acopos matrix are (5,7)
        Use moveit_offset to convert the coordinates to the moveit frame --> Robot arms EEF Poses.
        """
        ## TBD - Add robot end effector offsets

        if robot_arm is None:
            robot_arm = self.planning_group #if self.planning_group is not None else None

        lim_x = self.flyways_matrix.shape[1] * 2
        lim_y = self.flyways_matrix.shape[0] * 2

        # Moveit offset
        offset = [1.275, -1.04, 0.0]

        eef_offset = [0.0, 0.0, 0.0]

        if moveit_offset:
            x_coord = -(y_pos * 0.12 + 0.06) + offset[0]
            y_coord = (x_pos * 0.12 + 0.06) + offset[1]
        else:
            x_coord = x_pos * 0.12 + 0.06
            y_coord = y_pos * 0.12 + 0.06

        if x_pos > (lim_x - 1):
            raise ValueError("x_pos exceeds the size of the platform.")
        if y_pos > (lim_y - 1):   
            raise ValueError("y_pos exceeds the size of the platform.")
        
        return x_coord, y_coord


    def sim_xbots_movement_bfs(self, step_size):
        max_speed = 1.0 # m/s
        move_increment = step_size * max_speed 

        for xbot in range(self._number_shuttles):
            prim = self.prim_dict["prim_{}".format(xbot + 1)]
            current_pos = prim.GetAttribute('xformOp:translate').Get()

            # Convert current_pos and target to grid coordinates
            start = (int((current_pos[0] + 0.06)/0.12), len(self.grid) - 1 - int((current_pos[1] + 0.06)/0.12))  
            end = (int((self.targets_x[xbot] + (0.06 if self.targets_x[xbot] % 0.12 != 0 else 0))/0.12), 
                    len(self.grid) - 1 - int((self.targets_y[xbot] + (0.06 if self.targets_y[xbot] % 0.12 != 0 else 0))/0.12))

            path = self.bfs(self.grid, start, end)

            if path is not None and len(path) > 1:
                next_step = path[1]  # Take the second step in the path (first is current position)
                # Convert next_step from grid coordinates back to simulation's coordinates
                next_step_sim = ((next_step[0]*0.12) + (0.06 if next_step[0] % 1 != 0 else 0), 
                                ((len(self.grid) - 1 - next_step[1])*0.12) + (0.06 if (len(self.grid) - 1 - next_step[1]) % 1 != 0 else 0))
                prim.GetAttribute('xformOp:translate').Set((next_step_sim[0], next_step_sim[1], current_pos[2]))
                print("Moving xbot {} to {}".format(xbot + 1, next_step_sim))

    # BFS algorithm
    def bfs(self, grid, start, end):
        queue = []
        queue.append([start])  # Wrap the start tuple in a list
        visited = set(start)  # Create a set to store visited nodes

        while queue:
            path = queue.pop(0)
            node = path[-1]  # Get the last node in this path
            if node == end:
                return path

            for direction in [(0, 1), (0, -1), (1, 0), (-1, 0)]:  # Right, Left, Down, Up
                new_node = (node[0] + direction[0], node[1] + direction[1])

                if (new_node[0] >= 0 and new_node[0] < len(grid) and  # Check grid boundaries
                    new_node[1] >= 0 and new_node[1] < len(grid[0]) and
                    grid[new_node[0]][new_node[1]] == 1 and  # Check if new_node is walkable
                    new_node not in visited):  # Check if the node has not been visited

                    new_path = list(path)
                    new_path.append(new_node)
                    queue.append(new_path)
                    visited.add(new_node)  # Add the new node to the visited set

        print("No valid path found.")
        return None
# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
# Author: Daniel Moreno París (dmoren21@student.aau.dk) & Andrei Voica (avoica18@student.aau.dk)

from omni.isaac.examples.maps.maps_reader import ControllerJointsLoader
from omni.isaac.examples.maps.maps_reader import RecipeLoader

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core import World
from omni.isaac.core.prims import GeometryPrim, XFormPrim, RigidPrim
import omni.kit.commands
from pxr import Sdf, Gf, UsdPhysics
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles, euler_to_rot_matrix
import numpy as np

from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation, ArticulationSubset
from omni.isaac.core.physics_context import PhysicsContext

import carb

# PMC Library Imports
from pmclib import system_commands as sys   # PMC System related commands
from pmclib import xbot_commands as bot     # PMC Mover related commands
from pmclib import pmc_types                # PMC API Types
import time
import random
import functools


from omni.isaac.core.utils import viewports, extensions
from omni.isaac.core.utils.prims import set_targets

import asyncio
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion
import math

# import sys

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
        self._number_shuttles = 4
        # self._shuttle_position = np.array([1.2277, -0.9815, 1.07])
        self._shuttle_position = np.array([0.06, 0.06, 1.07]) #([0.06, 0.06, 1.07])
        self._platform_limits = np.array([0.0, 0.0, 0.832, 0.596]) # x_min, y_min, x_max, y_max
        self._shuttle_scale = 0.01
        # self.xbot_ids = [1, 2, 3, 4, 5, 6, 7, 8]
        self.xbot_ids = [i for i in range(1, self._number_shuttles + 1)] 

        self.targets_x = []
        self.targets_y = []

        # Trays
        self._number_tray_vial = 1
        self._tray_vial_position = np.array([0.35992, -0.15884, 1.063]) #([0.06, 0.06, 1.10])
        # self._tray_vial_position = np.array([1.2277, -1.2, 1])
        self._tray_vial_scale = 0.0098

        self._number_tray_beaker = 1
        # self._tray_beaker_position = np.array([0.30, 0.06, 1.090])
        self._tray_beaker_position = np.array([0.57916, -0.15884, 1.063])
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

        # Magnetic Stirrer
        self._stirrer_position = np.array([-0.15554, 0.71716, 1.0049]) 
        self._stirrer_orientation = np.array(euler_angles_to_quat([np.pi/2, 0, 0]))
        self._stirrer_scale = 1.0

        # HPLC
        #self._hplc_position = np.array([1.05725, -0.10274, 0.0369])
        self._hplc_position = np.array([-1.03033, 0.15418, 1.0049])  
        self._hplc_orientation = np.array(euler_angles_to_quat([0, 0, 0]))
        self._hplc_scale = 0.01
        
        # Loading station
        #self._loading_station_position = np.array([1.42345, -0.53634, 0.0369])  
        self._loading_station_position = np.array([-1.47442, 0.58184, 1.0049]) 
        self._loading_station_orientation = np.array(euler_angles_to_quat([0, 0, np.pi/2]))
        self._loading_station_scale = 0.01

        # Removing station
        #self._removing_station_position = np.array([1.15523, -1.15596, 0.0369])  
        self._removing_station_position = np.array([-1.04649, 1.23841, 1.0049]) 
        self._removing_station_orientation = np.array(euler_angles_to_quat([0, 0, 0]))
        self._removing_station_scale = 0.01

        # Kuka Multiple Arms:
        self._kuka_arms_position = np.array([0.0, 0.0, 1.0]) 
        self._kuka_arms_orientation = np.array(euler_angles_to_quat([0, 0, 0]))
        self._kuka_arms_scale = 1.0

        # Repository path:
        # self.repo_folder = "/home/andrei/P10-MAP/"
        self.repo_folder = "/home/robotlab/Documents/Github/P10-MAP/"

        # USD asset paths:
        # self.asset_folder = "omniverse://localhost/Projects/MAPs-AAU/Assets/"
        self.asset_folder = self.repo_folder + "assets/"
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
            "lab_setup": self.asset_folder + "Lab_setup_v0.usd", # Lab Setup without robots or Acopos Matrix
            "hplc": self.asset_folder + "Loading_station/Loading_station.usd",
            "loading_station": self.asset_folder + "Loading_station/Loading_station.usd",
            "removing_station": self.asset_folder + "Loading_station/Loading_station.usd"
        }


        # Prim paths Dictionaries:
        self.shuttles_prim_dict = {} # Dictionary to store shuttle prim paths
        self.items_prim_dict = {} # Dictionary to store tray vial prim paths
        self.eef_link_prim_dict = {} # Dictionary to store eef link prim paths for each robot
        self.gripper_prim_dict = {} # Dictionary to store gripper prim paths for each robot

        self.current_pos_dict = {} # Dictionary to store shuttle current positions


        # Get dictionary with planning group, joints and eef link for each robot
        self.filename = self.repo_folder + "src/kuka_config_multiple/config/simple_moveit_controllers.yaml"
        self.joints_loader = ControllerJointsLoader(self.filename)
        self.robot_joints_data = self.joints_loader.get_controller_data()

        print(self.robot_joints_data)

        # Load recipe with experiment instructions
        self.actions_file = self.repo_folder + "recipe/recipe_v2.yaml"
        self.actions_loader = RecipeLoader(self.actions_file)
        self.recipe = self.actions_loader.read_instructions_from_yaml()

        # Ros topics messages
        self.planning_group = String() # ROS topic name for move group
        self.joint_state_request = JointState()
        self.pose_request = Pose()  
        self.cartesian_path_request = PoseArray()

        self.action_completed = False
        self.start_time = 0
        self.action_times = {}

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

        # Reload recipe
        self.recipe = self.actions_loader.read_instructions_from_yaml()

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
            self.targets_x = np.append(self.targets_x, self._shuttle_position[0] + np.array([0.12 *i]))
            self.targets_y = np.append(self.targets_y, self._shuttle_position[1])
        
            
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

        # Add Magnetic Stirrer
        add_reference_to_stage(usd_path=self.asset_paths["stirrer"],
                                prim_path="/World/LabSetup/Stirrer")
        
        world.scene.add(RigidPrim(prim_path ="/World/LabSetup/Stirrer",
                                            name="magnetic_stirrer",
                                            position = self._stirrer_position,
                                            orientation = self._stirrer_orientation,
                                            mass = 3))

        # Add HPLC
        add_reference_to_stage(usd_path=self.asset_paths["hplc"],
                                prim_path="/World/LabSetup/hplc")
        
        world.scene.add(RigidPrim(prim_path ="/World/LabSetup/hplc",
                                            name="hplc",
                                            position = self._hplc_position,
                                            orientation = self._hplc_orientation,
                                            mass = 20))
        
        # Add Loading station
        add_reference_to_stage(usd_path=self.asset_paths["loading_station"],
                                prim_path="/World/LabSetup/loading_station")
        
        world.scene.add(RigidPrim(prim_path ="/World/LabSetup/loading_station",
                                            name="loading_station",
                                            position = self._loading_station_position,
                                            orientation = self._loading_station_orientation,
                                            mass = 20))
        
        # Add Remove station
        add_reference_to_stage(usd_path=self.asset_paths["removing_station"],
                                prim_path="/World/LabSetup/removing_station")
        
        world.scene.add(RigidPrim(prim_path ="/World/LabSetup/removing_station",
                                            name="removing_station",
                                            position = self._removing_station_position,
                                            orientation = self._removing_station_orientation,
                                            mass = 20))
        # Add Robots references
        add_reference_to_stage(usd_path=self.asset_paths["kuka_multiple"],
                                prim_path="/World/Kuka_Multiple_Arms")

        
        self.kukas = world.scene.add(Articulation(prim_path ="/World/Kuka_Multiple_Arms",
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
        self._world.scene.enable_bounding_boxes_computations()

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
                self.shuttles_prim_dict[key_name] = prim
            else:
                print("Error: shuttle prim not found at path {}".format(shuttle_path))

        # Items Prim Dictionary
        for tray_vial in range(self._number_tray_vial):
            tray_vial_path = "/World/LabSetup/Grid/tray_vial_{}".format(tray_vial + 1)
            prim = stage.GetPrimAtPath(tray_vial_path)
            if prim:
                key_name = "prim_tray_vial_{}".format(tray_vial + 1)
                self.items_prim_dict[key_name] = prim

        for tray_beaker in range(self._number_tray_beaker):
            tray_beaker_path = "/World/LabSetup/Grid/tray_beaker_{}".format(tray_beaker + 1)
            prim = stage.GetPrimAtPath(tray_beaker_path)
            if prim:
                key_name = "prim_tray_beaker_{}".format(tray_beaker + 1)
                while key_name in self.items_prim_dict:  # Check if the key already exists in the dictionary
                    tray_beaker += self._number_tray_vial  # Increment the index by the number of vials
                    tray_beaker_path = "/World/LabSetup/Grid/tray_beaker_{}".format(tray_beaker + 1)
                    prim = stage.GetPrimAtPath(tray_beaker_path)
                    key_name = "prim_{}_beaker".format(tray_beaker + 1)
                self.items_prim_dict[key_name] = prim

        print("ITEMS DICT: ", self.items_prim_dict)

        # Iterate over each robot
        for robot_name, robot_data in self.robot_joints_data.items():
            eef_link = robot_data['eef_link']
            eef_link_path = "/World/Kuka_Multiple_Arms/{}".format(eef_link)
            prim = stage.GetPrimAtPath(eef_link_path)
            if prim:
                self.eef_link_prim_dict[robot_name] = prim
            else:
                print("Error: eef link prim not found at path {}".format(eef_link_path))

        print("EEF DICT: ", self.eef_link_prim_dict)

        
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

        # Control Switch
        if self.control_switch == 0:
            # self._world.add_physics_callback("sim_step_shuttles", callback_fn=self.sim_xbots_movement_2)

            # rospy.init_node('isaac_test', anonymous=True)
            # self.pub_joints = rospy.Publisher("/joint_command_desired", queue_size=1)
            #self.on_impulse_event()

            self._world.add_physics_callback("sim_step_impulse", callback_fn=self.on_impulse_event)
            # self._world.add_physics_callback("sim_step_shuttles", self.sim_xbots_movement)


        elif self.control_switch == 1:
            self._connect_pmc()  # Connect to PMC
            self._world.add_physics_callback("sim_step_read_acopos", callback_fn=self.read_xbots_positions) #callback names have to be unique
            self._world.add_physics_callback("sim_step_move_acopos", callback_fn=self.send_xbots_positions)
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
        # self._convexIncludeRel.AddTarget(self._table_ref_geom.prim_path)

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
        if world.physics_callback_exists("sim_step_read_acopos"):
            world.remove_physics_callback("sim_step_read_acopos")
        # world.add_physics_callback("sim_step_shuttles", self.sim_xbots_movement)
        await world.play_async()
        return
    
    async def _on_real_control_event_async(self):
        world = self.get_world()
        if world.physics_callback_exists("sim_step_shuttles"):
            world.remove_physics_callback("sim_step_shuttles")
        self._world.add_physics_callback("sim_step_read_acopos", callback_fn=self.read_xbots_positions) #callback names have to be unique
        #self._world.add_physics_callback("sim_step_move_acopos", callback_fn=self.send_xbots_positions) ## Random positions right now
        await world.play_async()
        return


    ## CONTROL FUNCTIONS
    # Function to move selected robot to desired joints position   
    def move_to_joint_state(self, planning_group, joint_state_request):

        moveit_planning_group = self.robot_joints_data[planning_group]["planning_group"]
        self.planning_group = planning_group

        self.joint_state_request.position = joint_state_request
        self.pub_group.publish(moveit_planning_group)
        self.pub_joints.publish(self.joint_state_request)

        if len(joint_state_request) == 6:
            # Add a physics callback to check when the action has been completed
            callback_fn = functools.partial(self.on_sim_step_check, planning_group, joint_state_request)
            self._world.add_physics_callback("sim_step_check", callback_fn)

        return
    
    # Function to move selected robot to desired pose 
    def move_to_pose(self, planning_group, position, orientation = [0.0, np.pi/2, 0.0]):

        moveit_planning_group = self.robot_joints_data[planning_group]["planning_group"]
        self.planning_group = planning_group

        quaternion = euler_angles_to_quat(orientation)  # reverse the order of the angles to be rxyz as in ROS
        self.pose_request = self.create_pose_msg(position, quaternion)
        self.pub_group.publish(moveit_planning_group)
        self.pub_pose.publish(self.pose_request)
     
        # Add a physics callback to check when the action has been completed
        callback_fn = functools.partial(self.on_sim_step_check, planning_group, position)
        self._world.add_physics_callback("sim_step_check", callback_fn)

        return   

    # Function to move selected robot to desired pose using cartesian path
    def move_along_cartesian_path(self, planning_group, waypoints):

        moveit_planning_group = self.robot_joints_data[planning_group]["planning_group"]
        self.planning_group = planning_group

        self.cartesian_path_request.header.stamp = rospy.Time.now()
        self.cartesian_path_request.header.frame_id = 'world'  # or whatever frame_id you are using

        for waypoint in waypoints:
            position, orientation = waypoint
            quaternion = euler_angles_to_quat(orientation)  # convert euler to quaternion
            pose = self.create_pose_msg(position, quaternion)
            self.cartesian_path_request.poses.append(pose)

        print("Cartesian path request: ", self.cartesian_path_request)
        self.pub_group.publish(moveit_planning_group)
        self.pub_cartesian_path.publish(self.cartesian_path_request)

        # Add a physics callback to check when the action has been completed
        self._world.add_physics_callback("sim_step_check", lambda arg: self.on_sim_step_check(planning_group, position))

        return

    # Function to move selected selected shuttle to desired position
    def move_shuttle_to_target(self, xbot_id: int , target_x, target_y):
        # Check if the xbot_id exists in xbot_ids
        if xbot_id in self.xbot_ids:
            # Update the corresponding target_x and target_y values
            self.targets_x[xbot_id - 1] = target_x
            self.targets_y[xbot_id - 1] = target_y
        else:
            # If the xbot_id doesn't exist in xbot_ids, raise an exception
            raise ValueError(f"xbot_id {xbot_id} not found in xbot_ids")
        
        desired_position = [target_x, target_y]
        print("xbot_id_function: ", xbot_id)
        print("desired_position: ", desired_position)

        # Add physics callback to move the selected shuttle to the desired position 
        # and to check when the action has been completed
        self._world.add_physics_callback("sim_step_shuttles", self.sim_xbots_movement)

        callback_fn = functools.partial(self.on_sim_step_check, xbot_id, desired_position)
        self._world.add_physics_callback("sim_step_check", callback_fn)

        # self._world.add_physics_callback("sim_step_check", lambda xbot_id=xbot_id, desired_position=desired_position: self.on_sim_step_check(xbot_id, desired_position))

    def attach_object(self, planning_group, state, item):
        # Attach the shuttle to the robot arm
        callback_fn = functools.partial(self.on_sim_attach_object, planning_group, state, item)
        self._world.add_physics_callback("sim_attach_object", callback_fn)

    def on_sim_attach_object(self, planning_group, state, item, step_size = 0.01):
        # Get prim of item
        prim_item = self.items_prim_dict['prim_{}'.format(item)]

        # Get prim of robot arm
        offset = 0.02
        if isinstance(planning_group, int):
            if state == True:

                shuttle_pos = self.get_shuttle_position(planning_group)
                prim_item.GetAttribute('xformOp:translate').Set(( shuttle_pos[0], shuttle_pos[1] , shuttle_pos[2] + offset ))
                
        elif isinstance(planning_group, str):
                # eef_pos, eef_orient = self.get_eef_link_position(planning_group)
                print("Not working")
                # # Set the position of the item
                # item_pos = prim_item.GetAttribute('xformOp:translate').Get()
                # prim_item.GetAttribute('xformOp:translate').Set(( eef_pos[1] + 1.03443 , -eef_pos[0] +1.46063 -0.18, eef_pos[2] + 1 ))

                # # CHECK Maybe create a fake point in between both grippers to attach the item to the robot arm

                # # Convert orientation from GfQuatd to GfQuatf
                # # eef_orient_f = Gf.Quatf(eef_orient)
                # # print(eef_orient_f)
                # # prim_item.GetAttribute('xformOp:orient').Set(eef_orient_f)

                # # Transform orientation from euler angles to quaternion
                # quat_prim = euler_angles_to_quat([0.0, 0.0, 0.0])
                # quat = Gf.Quatf(*quat_prim)
                # # Set Orientation of item
                # prim_item.GetAttribute('xformOp:orient').Set(quat)



    # Function to open and close the gripper
    def gripper_control(self, planning_group, state):
        if state == "open":
            self.move_to_joint_state(planning_group, [0.0 , 0.0])
        elif state == "close":
            self.move_to_joint_state(planning_group, [-0.0030 , -0.0030])
        else:
            raise ValueError(f"state {state} not found in gripper_control")
        
        desired_position = state
        callback_fn = functools.partial(self.on_sim_step_check, planning_group, desired_position)
        self._world.add_physics_callback("sim_step_check", callback_fn)


    ## GET DATA FUNCTIONS
    def get_eef_link_position(self, robot_arm):
        # Collect end effector position
        prim_eef_link = self.eef_link_prim_dict[robot_arm] # Robot arm is a string (e.g. "robot_arm_1"")

        eef_pos = prim_eef_link.GetAttribute('xformOp:translate').Get()
        eef_orient = prim_eef_link.GetAttribute('xformOp:orient').Get()


        return eef_pos[0], eef_pos[1], eef_pos[2], eef_orient
        # return eef_pos, eef_orient

    def get_shuttle_position(self, xbot_id):
        # Retrieve the shuttle position
        shuttle_prim = self.shuttles_prim_dict["prim_{}".format(xbot_id)]
        shuttle_pos = shuttle_prim.GetAttribute('xformOp:translate').Get()
        return shuttle_pos[0], shuttle_pos[1], shuttle_pos[2]
        

    def get_gripper_joints_position(self, robot_hand):
        """ Get the current joint positions of the robot gripper """
        # Create an ArticulationSubset instance
        articulation_subset = ArticulationSubset(articulation=self.kukas, joint_names=self.robot_joints_data[robot_hand]['joints'])
        # Get the joint positions
        joint_pos_left, joint_pos_right = articulation_subset.get_joint_positions()

        return joint_pos_left, joint_pos_right

    def get_joints_position(self, robot_arm):
        """ Get the current joint positions of the robot arm """
        articulation_subset = ArticulationSubset(articulation=self.kukas, joint_names=self.robot_joints_data[robot_arm]['joints'])
        current_joint_values = articulation_subset.get_joint_positions()

        # # Debugging:
        # current_joint_states = self.get_joints_position("robot_arm_1")
        # carb.log_warn("Current joint states {}: {}".format("robot_arm_1" ,repr(current_joint_states)))
        # current_joint_states = self.get_joints_position("robot_arm_2")
        # carb.log_warn("Current joint states {}: {}".format("robot_arm_2" ,repr(current_joint_states)))
        # current_joint_states = self.get_joints_position("robot_arm_3")
        # carb.log_warn("Current joint states {}: {}".format("robot_arm_3" ,repr(current_joint_states)))
        # current_joint_states = self.get_joints_position("robot_arm_4")
        # carb.log_warn("Current joint states {}: {}".format("robot_arm_4" ,repr(current_joint_states)))
        # current_joint_states = self.get_joints_position("robot_arm_5")
        # carb.log_warn("Current joint states {}: {}".format("robot_arm_5" ,repr(current_joint_states)))
        

        return current_joint_values


    def has_reached_position(self, planning_group, desired_position, tolerance=0.01):
        """
        Check if the robot/shuttle has reached the desired position
        planning_group: string (Robot Arm) or integer (Shuttle)
        desired_position: list of 3 floats [x, y, z] in Isaac Sim coordinates or string ("open" or "close") for gripper
        """

        if isinstance(desired_position, str) and desired_position in ["open", "close"]:
            # Get the gripper joint positions
            joint_pos_left, joint_pos_right = self.get_gripper_joints_position(planning_group)
            print("Joint positions: ", joint_pos_left, joint_pos_right)
            
            # Check if the gripper is open or closed
            if (desired_position == "close" and (joint_pos_left < -0.0010 or joint_pos_right < -0.0010)) or \
            (desired_position == "open" and (joint_pos_left > -0.0001 and joint_pos_right > -0.0001)):
                elapsed_time = time.time() - self.start_time
                action_name = "{} gripper {}".format(planning_group, desired_position)
                self.action_times.setdefault(action_name, []).append(elapsed_time)
                carb.log_warn("{} completed in {:.3f} seconds".format(action_name, elapsed_time))
                self.action_completed = True # Set the action_completed flag to True
                return True
            else:
                print("Current gripper position: ", joint_pos_left, joint_pos_right)
                return False

        elif isinstance(planning_group, str):  # Get current position of the robot

            if len(desired_position) == 3: # Move to Pose checking (eef position)
                current_position = self.get_eef_link_position(planning_group)
                # Compute the distance between the current position and the desired position
                distance = math.sqrt((current_position[0] - desired_position[0])**2 +
                                     (current_position[1] - desired_position[1])**2 +
                                     (current_position[2] - desired_position[2])**2)
                # Check if the distance is within the tolerance
                if distance <= tolerance:
                    elapsed_time = time.time() - self.start_time
                    action_name = "{} moved to target".format(planning_group)
                    self.action_times.setdefault(action_name, []).append(elapsed_time)
                    carb.log_warn("{} completed in {:.3f} seconds".format(action_name, elapsed_time))
                    self.action_completed = True # Set the action_completed flag to True
                    
                    current_joint_states = self.get_joints_position(planning_group)
                    carb.log_warn("Current joint states: {}".format(repr(current_joint_states)))
                    return True
                else:
                    print("Current position: ", current_position)   
                    print("Distance: ", distance)
                    return False
                
            elif len(desired_position) == 6: # Move to Joint States checking (joint positions)
                current_joint_states = self.get_joints_position(planning_group)
                # Compute the distance between the current position and the desired position
                distance = math.sqrt((current_joint_states[0] - desired_position[0])**2 +
                                     (current_joint_states[1] - desired_position[1])**2 +
                                     (current_joint_states[2] - desired_position[2])**2 +
                                     (current_joint_states[3] - desired_position[3])**2 +
                                     (current_joint_states[4] - desired_position[4])**2 +
                                     (current_joint_states[5] - desired_position[5])**2)
                # Check if the distance is within the tolerance
                if distance <= tolerance:
                    elapsed_time = time.time() - self.start_time
                    action_name = "{} moved to joint states".format(planning_group)
                    self.action_times.setdefault(action_name, []).append(elapsed_time)
                    carb.log_warn("{} completed in {:.3f} seconds".format(action_name, elapsed_time))
                    self.action_completed = True
                else:
                    print("Current position: ", current_joint_states)   
                    print("Distance: ", distance)
                    return False
            else:
                print("Invalid desired position")
                return False

        if isinstance(planning_group, int):
            # Get current position of the shuttle
            current_position = self.get_shuttle_position(planning_group)
            # Compute the distance between the current position and the desired position
            distance = math.sqrt((current_position[0] - desired_position[0])**2 +
                                 (current_position[1] - desired_position[1])**2)
            # Check if the distance is within the tolerance
            if distance <= tolerance:
                elapsed_time = time.time() - self.start_time
                action_name = "Shuttle {} moved to target".format(planning_group)
                self.action_times.setdefault(action_name, []).append(elapsed_time)
                carb.log_warn("{} completed in {:.3f} seconds".format(action_name, elapsed_time))
                self.action_completed = True # Set the action_completed flag to True
                return True
            else:
                print("Current position: ", current_position)   
                print("Distance: ", distance)
                return False

        else:
            print("Invalid planning group type. Must be a string for a robot arm or integer for a shuttle.")
            return False
    
    def print_action_times_summary(self):
        # Print the sum of times for each action, ordered by the number of actions
        for action, times in sorted(self.action_times.items(), key=lambda item: len(item[1])):
            carb.log_warn("Action '{}': completed {} times, total time {:.3f} seconds".format(action, len(times), sum(times)))
        
        
    def execute_actions(self):
        if len(self.recipe) > 0:
            if self.action_completed:
                action = self.recipe.pop(0)  # Retrieve the first action in the list
                action_name = action['action']
                parameters = action['parameters']

                self.start_time = time.time()  # Record the start time for each action

                # Print the action to be executed with its parameters
                carb.log_warn("Executing action: " + action_name + ": " + str(parameters))

                world= self.get_world()
                if world.physics_callback_exists("sim_step_check"):
                    world.remove_physics_callback("sim_step_check")
                if world.physics_callback_exists("sim_step_shuttles"):
                    world.remove_physics_callback("sim_step_shuttles")

                self.action_completed = False # Set the action_completed flag to False

                if action_name == 'MOVE_TO_JOINT_STATE':
                    self.move_to_joint_state(**parameters)  # The ** operator is used to unpack the dictionary into keyword arguments
                
                elif action_name == 'MOVE_TO_POSE_MOVEIT':
                    self.move_to_pose(**parameters)  # The ** operator is used to unpack the dictionary into keyword arguments
                
                elif action_name == 'MOVE_TO_POSE_IN_PLATFORM':
                    position_xy = self.platform_pos_to_coordinates(parameters['position'][0],parameters['position'][1], moveit_offset = True)
                    self.move_to_pose(parameters['planning_group'], [position_xy[0],position_xy[1],parameters['position'][2]], parameters['orientation'])
                
                elif action_name == 'MOVE_ALONG_CARTESIAN_PATH':
                    self.move_along_cartesian_path(**parameters)

                elif action_name == 'MOVE_SHUTTLE_TO_TARGET':
                    position_xy = self.platform_pos_to_coordinates(parameters['target_x'],parameters['target_y'], moveit_offset = False)
                    xbot_id = int(parameters['xbot_id'])
                    self.move_shuttle_to_target(xbot_id, position_xy[0], position_xy[1])
                
                elif action_name == 'GRIPPER_CONTROL':
                    self.gripper_control(**parameters)
                
                elif action_name == 'ATTACH_OBJECT':
                    if parameters['state'] == True:
                        self.attach_object(**parameters)
                    elif parameters['state'] == False:
                        world= self.get_world()
                        if world.physics_callback_exists("sim_attach_object"):
                            world.remove_physics_callback("sim_attach_object") # Remove the physics callback
                    self.action_completed = True

                else:
                    print("Invalid action name: ", action_name)

        else:
            world= self.get_world()
            if world.physics_callback_exists("sim_step_auto_play"):
                world.remove_physics_callback("sim_step_auto_play")            
            if world.physics_callback_exists("sim_step_check"):
                world.remove_physics_callback("sim_step_check")
            if world.physics_callback_exists("sim_step_shuttles"):
                world.remove_physics_callback("sim_step_shuttles")
            carb.log_warn("Recipe completed!")
            self.print_action_times_summary()



    async def _on_start_experiment_event_async(self):

        self.action_completed = True

        self._world.add_physics_callback("sim_step_auto_play", callback_fn=self.on_automatic_execution)
        #self.execute_actions()

        return

    def on_impulse_event(self, step_size):
        # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
        og.Controller.set(og.Controller.attribute("/World/Kuka_Multiple_Arms/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)
        
    def on_sim_step_check(self, planning_group, desired_position, step_size=1):
        # Check if the robot has reached the desired position
        self.has_reached_position(planning_group, desired_position)

    def on_automatic_execution(self, step_size=1):
        # Execute the actions in the recipe
        self.execute_actions()


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

        max_speed = 1.0 # m/s
        move_increment = step_size * max_speed 

        for shuttle_number in range(self._number_shuttles):
            prim = self.shuttles_prim_dict["prim_{}".format(shuttle_number + 1)]

            current_pos = prim.GetAttribute('xformOp:translate').Get() 

            #Move shuttle up
            if (self.targets_y[shuttle_number]) > current_pos[1]:
                prim.GetAttribute('xformOp:translate').Set((current_pos[0], current_pos[1] + move_increment, current_pos[2]))
                if (current_pos[1] + move_increment) > self.targets_y[shuttle_number]:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0], self.targets_y[shuttle_number], current_pos[2])) 
            # Move shuttle down
            elif (self.targets_y[shuttle_number]) < current_pos[1]:
                prim.GetAttribute('xformOp:translate').Set((current_pos[0], current_pos[1] - move_increment, current_pos[2]))
                if (current_pos[1] - move_increment) < self.targets_y[shuttle_number]:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0], self.targets_y[shuttle_number], current_pos[2]))

            # check if we reached the target in y axis, then start moving in x direction.
            if abs(current_pos[1] - self.targets_y[shuttle_number]) < move_increment:
                # Move shuttle right
                if (self.targets_x[shuttle_number]) > current_pos[0]:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0] + move_increment, current_pos[1], current_pos[2]))
                    if (current_pos[0] + move_increment) > self.targets_x[shuttle_number]:
                        prim.GetAttribute('xformOp:translate').Set((self.targets_x[shuttle_number], current_pos[1], current_pos[2]))
                # Move shuttle left
                elif (self.targets_x[shuttle_number]) < current_pos[0]:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0] - move_increment, current_pos[1], current_pos[2]))
                    if (current_pos[0] - move_increment) < self.targets_x[shuttle_number]:
                        prim.GetAttribute('xformOp:translate').Set((self.targets_x[shuttle_number], current_pos[1], current_pos[2]))
    
    # Move xbots in simulation (Checking other shuttles in the path)
    def sim_xbots_movement_collision(self, step_size):

        max_speed = 1.0 # m/s
        move_increment = step_size * max_speed 

        for xbot in range(self._number_shuttles):

            prim = self.shuttles_prim_dict["prim_{}".format(xbot + 1)]

            current_pos = prim.GetAttribute('xformOp:translate').Get()

            x_pos_control = []
            y_pos_control = []

            # Collect all shuttles positions
            for shuttle_number in range(self._number_shuttles):
                prim_others = self.shuttles_prim_dict["prim_{}".format(shuttle_number + 1)]
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
            prim = self.shuttles_prim_dict["prim_{}".format(shuttle_number + 1)]

            # Set position of shuttle
            prim.GetAttribute('xformOp:translate').Set((xbot_positions[shuttle_number][0],
                                                        xbot_positions[shuttle_number][1] ,
                                                        xbot_positions[shuttle_number][2] + 1.06))
 
            # Transform orientation from euler angles to quaternion
            quat_prim = (euler_angles_to_quat([xbot_positions[shuttle_number][3],
                                               xbot_positions[shuttle_number][4],
                                               xbot_positions[shuttle_number][5]]))
            # quat = Gf.Quatd(*quat_prim)
            quat = Gf.Quatf(*quat_prim)


            # Set Orientation of shuttle
            prim.GetAttribute('xformOp:orient').Set(quat)


    def send_xbots_positions(self, step_size):
        """Send commands to the Xbots to move them to the next target position.
            Planning is done using PMC algorithm."""

        # Only update the Xbots if at least some time (s) have passed since the last update
        if time.time() - self.last_update_time >= 0.5:

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
                bot.auto_driving_motion_si(self._number_shuttles, xbot_ids=self.xbot_ids, targets_x=targets_x, targets_y=targets_y)
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
        """Create random coordinates for each shuttle in num_shuttles"""
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
        """Connect to PMC and gain mastership"""

        # Connect to PMC
        sys.auto_connect_to_pmc()
        if not sys.auto_connect_to_pmc():
            sys.connect_to_pmc("192.168.10.100") #sys.auto_connect_to_pmc()

        carb.log_warn("Connected: " + str(sys.auto_connect_to_pmc()))
        carb.log_warn("Status: " + str(sys.get_pmc_status()))

        # Gain mastership
        if not sys.is_master():
            sys.gain_mastership()

        carb.log_warn("Master: " + str(sys.is_master()))

        # Activate xBots
        bot.activate_xbots()


    def create_pose_msg(self, position, orientation, frame_id=''):
        """Create a ROS Pose message object from a position and orientation"""
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


##############################################################################################################
########################################## BFS Algortihm #####################################################
##############################################################################################################

    def sim_xbots_movement_bfs(self, step_size):
        max_speed = 1.0 # m/s
        move_increment = step_size * max_speed 

        for xbot in range(self._number_shuttles):
            prim = self.shuttles_prim_dict["prim_{}".format(xbot + 1)]
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
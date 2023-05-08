# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core import World
from omni.isaac.core.prims import GeometryPrim, XFormPrim
import omni.kit.commands
from pxr import Sdf, Gf, UsdPhysics
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import numpy as np

from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import Articulation

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
from math import tau

import sys

# Action graph imports
import omni.graph.core as og

# import rosgraph
#########################################################################################################


# KUKA_STAGE_PATH = "/World/Kuka_kr3_1"
# enable ROS bridge extension
extensions.enable_extension("omni.isaac.ros_bridge")

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage

# if not rosgraph.is_master_online():
#     carb.log_error("Please run roscore before executing this script")

#     exit()

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
        self._shuttle_position = np.array([1.2277, -0.9815, 1.07])
        #self._shuttle_position = np.array([0.0, 0.0, 1.07])
        self._platform_limits = np.array([0.0, 0.0, 0.832, 0.596]) # x_min, y_min, x_max, y_max
        self._target = np.array([0.8, 0.52])
        self._shuttle_scale = 0.01
        # self.xbot_ids = [1, 2, 3, 4, 5, 6, 7, 8]
        self.xbot_ids = [i for i in range(1, self._number_shuttles + 1)] 

        # Trays
        self._number_trayVial = 4
        self._trayVial_position = np.array([1.2277, -1.2, 1])
        self._trayVial_scale = 0.01

        self._number_trayFlask = 4
        self._trayFlask_position = np.array([1.42, -1.2, 1])
        self._trayFlask_scale = 0.01

        # Flyways:
        # DEFINE FLYWAYS MATRIX
        self.flyways_matrix = [[1, 1, 1],
                               [1, 1, 1],
                               [1, 1, 1], 
                               [1, 1, 1]]
        
        self._flyway_position = np.array([1.165, -0.92398, 0.99302])
        self._flyway_orientation = np.array([0, 0, 0, 1])
        self._flyway_scale = 0.01

        # Kuka Multiple Arms:
        self._kuka_arms_position = np.array([0.0, 0.0, 1.0])  # Gf.Vec3f(0.5, 0.0, 0.0)
        self._kuka_arms_orientation = np.array(euler_angles_to_quat([0, 0, 0]))
        self._kuka_arms_scale = 1.0

       
        # USD asset paths:
        #self.asset_folder = "omniverse://localhost/Projects/MAPs-AAU/Assets/"
        self.asset_folder = "/home/robotlab/Documents/Github/P10-MAP/assets/"
        self.asset_paths = {
            #"kr3": self.asset_folder + "kr3r540/kr3r540_v3/kr3r540_v3.usd",
            #"kr3": self.asset_folder + "kr3r540/kr3r540_v4/kr3r540_v4.usd", # Schunk Kr3
            "kr3": self.asset_folder + "kr3r540_v4/kr3r540_v4g.usd", # Schunk Kr3
            "kr4": self.asset_folder + "kr4r600/kr4r600_v2.usd", 
            "kuka_multiple": self.asset_folder + "kuka_multiple_arms/kuka_multiple_arms.usd",
            "franka": "omniverse://localhost/NVIDIA/Assets/Isaac/2022.2.1/Isaac/Robots/Franka/franka_alt_fingers.usd",
            "flyway": self.asset_folder + "flyways/flyway_segment.usd",
            # "shuttle": self.asset_folder + "120x120x10/acopos_shuttle_120.usd", # Basic shuttle
            "shuttle": self.asset_folder + "120x120x10/shuttle.usd",
            "tray_vial" : self.asset_folder + "Trays/Tray_vial.usd",
            "tray_flask" : self.asset_folder + "Trays/Tray_flask.usd",
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

        # Define rospy topic names
        self.joint_state_request = JointState()
        self.joint_state_request.name = ["joint_a1", "joint_a2","joint_a3", "joint_a4", "joint_a5","joint_a6"]

        self.control_switch = 0 # 0: Sim, 1: PMC

        return

    # This function is called to setup the assets in the scene for the first time
    # Class variables should not be assigned here, since this function is not called
    # after a hot-reload, its only called to load the world starting from an EMPTY stage
    def setup_scene(self):
        # A world is defined in the BaseSample, can be accessed everywhere EXCEPT __init__
        world = self.get_world()
        world = World.instance()

        # stage.SetDefaultPrim(world)
        world.scene.add_default_ground_plane() # adds a default ground plane to the scene

        # Add Xform reference for the shuttles
        world.scene.add(XFormPrim(prim_path="/World/LabSetup", name=f"LabSetup"))

        # # Add Lab Setup Reference
        # add_reference_to_stage(usd_path=self.asset_paths["lab_setup"], prim_path="/World/LabSetup")
        # world.scene.add(GeometryPrim(prim_path="/World/LabSetup", name=f"lab_setup_ref_geom", collision=True))

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
            world.scene.add(GeometryPrim(prim_path="/World/LabSetup/Grid/shuttle_{}".format(i+1),
                                         name="shuttle_{}_ref_geom".format(i+1), collision=True))
            
        # Add Trays
        for i in range(self._number_trayVial):
            add_reference_to_stage(usd_path=self.asset_paths["tray_vial"], prim_path="/World/LabSetup/Grid/trayVial_{}".format(i+1))
            world.scene.add(GeometryPrim(prim_path="/World/LabSetup/Grid/trayVial_{}".format(i+1),
                                         name="trayVial_{}_ref_geom".format(i+1), collision=True))

        for i in range(self._number_trayFlask):
            add_reference_to_stage(usd_path=self.asset_paths["tray_flask"], prim_path="/World/LabSetup/Grid/trayFlask_{}".format(i+1))
            world.scene.add(GeometryPrim(prim_path="/World/LabSetup/Grid/trayFlask_{}".format(i+1),
                                         name="trayFlask_{}_ref_geom".format(i+1), collision=True))
            
        # # Add Xform reference for each station
        # for i in range(len(self.station_info)):

        #     # Add Robots references
        #     add_reference_to_stage(usd_path=self.station_info[i+1]['asset'],
        #                             prim_path="/World/Kuka_{}_{}".format(self.station_info[i+1]['asset_name'], (i+1)))

            
        #     world.scene.add(Articulation(prim_path ="/World/Kuka_{}_{}".format(self.station_info[i+1]['asset_name'], (i+1)),
        #                                       name="Kuka_{}".format(i+1),
        #                                       position = self.station_info[i+1]['position'],
        #                                       orientation = self.station_info[i+1]['orientation']))
            
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
        for i in range(self._number_shuttles):
            await self._add_shuttle(i)
        for i in range(self._number_trayVial):
            await self._add_trayVial(i)
        for i in range(self._number_trayFlask):
            await self._add_trayFlask(i)
        # for i in range(len(self.station_info)):
        #     await self._add_station(i)



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
        self.pub = rospy.Publisher('/kr3_1/joint_command_isaac', JointState, queue_size=10)
        # self.pub2 = rospy.Publisher('/kr3_2/joint_command_isaac', JointState, queue_size=10)

        self.joint_state_request.position = [0 , -1 , -tau/8 , -tau/4 , 0 , tau/6 ]


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
            self.targets_x, self.targets_y = self.create_random_coordinates(self._number_shuttles)
            # self._world.add_physics_callback("sim_step", callback_fn=self.sim_xbots_movement_2)

            # rospy.init_node('isaac_test', anonymous=True)
            # self.pub = rospy.Publisher("/joint_command_desired", queue_size=1)
            #self.on_impulse_event()

            self._world.add_physics_callback("sim_step_impulse", callback_fn=self.on_impulse_event)

            # self._world.add_physics_callback("sim_step_ros", callback_fn=self.ros_tests)

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

    # Add shuttles to the scene
    async def _add_shuttle(self, shuttle_number):
        self._shuttle_ref_geom = self._world.scene.get_object(f"shuttle_{shuttle_number+1}_ref_geom")
        self._shuttle_ref_geom.set_local_scale(np.array([self._shuttle_scale]))
        self._shuttle_ref_geom.set_world_pose(position= self._shuttle_position + (-0.121 * (shuttle_number), 0, 0))
        self._shuttle_ref_geom.set_default_state(position=self._shuttle_position)
        self._shuttle_ref_geom.set_collision_approximation("none")

    # Add trays to the scene
    async def _add_trayVial(self, trayVial_number):
        self._trayVial_ref_geom = self._world.scene.get_object(f"trayVial_{trayVial_number+1}_ref_geom")
        self._trayVial_ref_geom.set_local_scale(np.array([self._trayVial_scale]))
        self._trayVial_ref_geom.set_world_pose(position= self._trayVial_position + (0, 0, 0.015 * (trayVial_number)))
        self._trayVial_ref_geom.set_default_state(position=self._trayVial_position)
        self._trayVial_ref_geom.set_collision_approximation("none")

    async def _add_trayFlask(self, trayFlask_number):
        self._trayFlask_ref_geom = self._world.scene.get_object(f"trayFlask_{trayFlask_number+1}_ref_geom")
        self._trayFlask_ref_geom.set_local_scale(np.array([self._trayFlask_scale]))
        self._trayFlask_ref_geom.set_world_pose(position= self._trayFlask_position + (0, 0, 0.021 * (trayFlask_number)))
        self._trayFlask_ref_geom.set_default_state(position=self._trayFlask_position)
        self._trayFlask_ref_geom.set_collision_approximation("none")

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
    
    async def _on_start_experiment_event_async(self):
        # Publish Joints for the robot
        # # self.pub.publish(self.joint_state_request)
        self.on_impulse_event()
        self.joint_state_request.position = [0 , -1 , -tau/2 , -tau/4 , 0 , tau/6 ]


        return

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
    
    def on_impulse_event(self, step_size):
        # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame

        self.pub.publish(self.joint_state_request)
        # print("Joint State Request ISAAC Published")

        og.Controller.set(og.Controller.attribute("/World/Kuka_kr3_1/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)
        og.Controller.set(og.Controller.attribute("/World/Kuka_kr3_2/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)
        og.Controller.set(og.Controller.attribute("/World/Kuka_kr3_3/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)
        og.Controller.set(og.Controller.attribute("/World/Kuka_kr3_4/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)

    # Move xbots in simulation (No collision detection)
    def sim_xbots_movement(self, step_size):

        max_speed = 3.0 # m/s
        max_accel = 10.0 # m/s^2
        move_increment = step_size * max_speed 

        for shuttle_number in range(self._number_shuttles):
            prim = self.prim_dict["prim_{}".format(shuttle_number + 1)]

            current_pos = prim.GetAttribute('xformOp:translate').Get() 

            print("current pos: ", current_pos)
            print("target_x", self.targets_x[shuttle_number], "shuttle_number: ", shuttle_number + 1)

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
    def sim_xbots_movement_2(self, step_size):

        # og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)


        max_speed = 1.0 # m/s
        move_increment = step_size * max_speed 



        for xbot in range(self._number_shuttles):

            prim = self.prim_dict["prim_{}".format(xbot + 1)]

            current_pos = prim.GetAttribute('xformOp:translate').Get()
 
            # FIRST DO IT HERE FOR EVERY SHUTTLE, AFTER JUST UPDATE THE UPDATED VALUE

            x_pos_control = []
            y_pos_control = []

            # Collect all shuttles positions
            for shuttle_number in range(self._number_shuttles):
                prim_others = self.prim_dict["prim_{}".format(shuttle_number + 1)]
                shuttles_pos = prim_others.GetAttribute('xformOp:translate').Get()
                print("shuttles_pos: ", shuttles_pos)
                x_pos_control.append(shuttles_pos[0])
                y_pos_control.append(shuttles_pos[1])

            print("x_pos_control: ", x_pos_control)
            print("y_pos_control: ", y_pos_control)

            # CHECK if condition
            for shuttle in range(self._number_shuttles):
                if xbot != shuttle and (current_pos[1] > (y_pos_control[shuttle] - 0.0602 - move_increment) or current_pos[1] < (y_pos_control[shuttle] + 0.0602 + move_increment) and current_pos[0] > (x_pos_control[shuttle] - 0.0602 - move_increment) or current_pos[0] < (x_pos_control[shuttle] + 0.0602 + move_increment)):                
                    continue_flag = True
                    print("continue_flag: ", continue_flag)
            if continue_flag:
                continue
            
            # WHEN THIS HAPPENS, BREAK THE OUTER LOOP -- TBD
        
            # Move shuttle right
            if (self.targets_x[xbot]) > current_pos[0]:
                prim.GetAttribute('xformOp:translate').Set((current_pos) + (move_increment, 0.0, 0.0))
                if (current_pos[0] + move_increment) > self.targets_x[xbot]:
                    prim.GetAttribute('xformOp:translate').Set((self.targets_x[xbot], current_pos[1], current_pos[2]))
                break
            # Move shuttle left
            elif (self.targets_x[xbot]) < current_pos[0]:
                prim.GetAttribute('xformOp:translate').Set((current_pos) - (move_increment, 0.0 , 0.0))
                if (current_pos[0] - move_increment) < self.targets_x[xbot]:
                    prim.GetAttribute('xformOp:translate').Set((self.targets_x[xbot], current_pos[1], current_pos[2]))
                break

            # for shuttle in range(self._number_shuttles):
            #     if xbot != shuttle:
            #         continue 
            #     elif (current_pos[0] > (x_pos_control[shuttle] - 0.0602 - move_increment) or current_pos[0] < (x_pos_control[shuttle] + 0.0602 + move_increment)):                   
            #         break

            for shuttle in range(self._number_shuttles):
                if xbot != shuttle and (current_pos[0] > (x_pos_control[shuttle] - 0.0602 - move_increment) or current_pos[0] < (x_pos_control[shuttle] + 0.0602 + move_increment)):                   
                    continue_flag = True
            if continue_flag:
                continue
                         
            # Move shuttle up
            if (self.targets_y[xbot]) > current_pos[1]:
                prim.GetAttribute('xformOp:translate').Set((current_pos) + (0.0, move_increment, 0.0))
                if (current_pos[1] + move_increment) > self.targets_y[xbot]:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0], self.targets_y[xbot], current_pos[2]))
                break
            # Move shuttle down
            elif (self.targets_y[xbot]) < current_pos[1]:
                prim.GetAttribute('xformOp:translate').Set((current_pos) - (0.0, move_increment, 0.0))
                if (current_pos[1] - move_increment) < self.targets_y[xbot]:
                    prim.GetAttribute('xformOp:translate').Set((current_pos[0], self.targets_y[xbot], current_pos[2]))
                break
            
            # #current_pos = prim.GetAttribute('xformOp:translate').Get()


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

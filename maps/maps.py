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

from omni.isaac.core.utils.nucleus import get_assets_root_path, get_server_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

import carb

from pmclib import system_commands as sys   # PMC System related commands
from pmclib import xbot_commands as bot     # PMC Mover related commands
from pmclib import pmc_types                # PMC API Types
import time
import random


class MAPs(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        # Positions are relative to parents, so set them with reversed values
        # SCENE GEOMETRY
        # env (group) spacing:
        self._env_spacing = 2.0
        self.last_update_time = time.time()

        # Lab Setup:
        self._lab_setup_position = np.array([0.0, 0.0, 0.0])  # Gf.Vec3f(0.5, 0.0, 0.0)
        self._lab_setup_orientation = np.array([0, 0, 0, 1])
        self._lab_setup_scale = 1.0

        # Shuttles:
        self._number_shuttles = 8
        self._shuttle_position = np.array([1.2277, -0.9815, 1.07])
        self._platform_limits = np.array([0.0, 0.0, 0.832, 0.596]) # x_min, y_min, x_max, y_max
        self._target = np.array([0.8, 0.52])
        self._shuttle_scale = 0.01

        # Shuttles Grid:
        self._grid_position = np.array([1.2877, -1.0415, 0.0])
        shuttle_orientation = np.pi/2
        self._grid_orientation = np.array([np.cos(shuttle_orientation/2), 0, 0, np.sin(shuttle_orientation/2)]) #Rotates 90 degrees around z-axis

        # USD asset paths:
        self.asset_folder = "omniverse://localhost/Projects/MAPs-AAU/Assets/"
        self.asset_paths = {
            "acopos_setup": self.asset_folder + "AcoposEnv_v1.usd", # Table + Acopos Matrix
            "kr3": self.asset_folder + "kr3r540/kr3r540_v3/kr3r540_v3.usd",
            "kr4": self.asset_folder + "kr4r600/kr4r600_v1/kr4r600_v1.usd", 
            "shuttle": self.asset_folder + "120x120x10/acopos_shuttle_120.usd",
            #"lab_setup": self.asset_folder + "Lab_setup_v2.usd" # Lab Setup with robots
            "lab_setup": self.asset_folder + "Lab_setup_v1.usd"  # Lab Setup without robots
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

        self.control_switch = 0 # 0: Sim, 1: PMC

        return


    # This function is called to setup the assets in the scene for the first time
    # Class variables should not be assigned here, since this function is not called
    # after a hot-reload, its only called to load the world starting from an EMPTY stage
    def setup_scene(self):
        # A world is defined in the BaseSample, can be accessed everywhere EXCEPT __init__
        world = self.get_world()
        world = World.instance()
        world.scene.add_default_ground_plane() # adds a default ground plane to the scene

        # Add Lab Setup Reference
        add_reference_to_stage(usd_path=self.asset_paths["lab_setup"], prim_path="/World")
        world.scene.add(GeometryPrim(prim_path="/World", name=f"lab_setup_ref_geom", collision=True))

        # Add Xform reference for the shuttles
        world.scene.add(XFormPrim(prim_path="/World/LabSetup/Grid", name=f"Grid"))

        # Add Xform reference for each station
        for i in range(len(self.station_info)):
            world.scene.add(XFormPrim(prim_path="/World/LabSetup/Station_{}".format(i+1), name="Station_{}".format(i+1)))

            add_reference_to_stage(usd_path=self.station_info[i+1]['asset'],
                                   prim_path="/World/LabSetup/Station_{}/{}".format((i+1), self.station_info[i+1]['asset_name']))
            world.scene.add(GeometryPrim(prim_path="/World/LabSetup/Station_{}/{}".format(i+1, self.station_info[i+1]['asset_name']),
                                            name="station_{}_asset_ref_geom".format(i+1), collision=True))

        # Add shuttles references
        for i in range(self._number_shuttles):
            add_reference_to_stage(usd_path=self.asset_paths["shuttle"], prim_path="/World/LabSetup/Grid/shuttle_{}".format(i+1))
            world.scene.add(GeometryPrim(prim_path="/World/LabSetup/Grid/shuttle_{}".format(i+1),
                                         name="shuttle_{}_ref_geom".format(i+1), collision=True))

        return

    # Here we assign the class's variables this function is called after load button is pressed
    # regardless starting from an empty stage or not this is called after setup_scene and after
    # one physics time step to propagate appropriate physics handles which are needed to retrieve
    # many physical properties of the different objects
    async def setup_post_load(self):

        # Load World and Assets
        self._world = self.get_world()
        self._world.scene.enable_bounding_boxes_computations()

        # Add USD Assets
        await self._add_lab_setup()
        await self._add_shuttles_grid()
        for i in range(len(self.station_info)):
            await self._add_station(i)
        for i in range(self._number_shuttles):
            await self._add_shuttle(i)


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


        # Control Switch
        if self.control_switch == 0:
            self._world.add_physics_callback("sim_step", callback_fn=self.sim_xbots_movement)
        elif self.control_switch == 1:
            self.connect_pmc()  # Connect to PMC
            self._world.add_physics_callback("sim_step", callback_fn=self.read_xbots_positions) #callback names have to be unique
            self._world.add_physics_callback("sim_step_move", callback_fn=self.send_xbots_positions)

        return

        # Add Lab Setup reference
    async def _add_lab_setup(self):
        self._lab_setup_ref_geom = self._world.scene.get_object(f"lab_setup_ref_geom")
        self._lab_setup_ref_geom.set_local_scale(np.array([self._lab_setup_scale]))
        self._lab_setup_ref_geom.set_world_pose(position=self._lab_setup_position,
                                                orientation=self._lab_setup_orientation)
        self._lab_setup_ref_geom.set_default_state(position=self._lab_setup_position,
                                                orientation=self._lab_setup_orientation)
        # lb = self._world.scene.compute_object_AABB(name=f"lab_setup_ref_geom")
        # zmin = lb[0][2]
        # zmax = lb[1][2]
        # self._lab_setup_position[2] = -zmin
        # self._lab_setup_height = zmax
        self._lab_setup_ref_geom.set_collision_approximation("none")
        #self._convexIncludeRel.AddTarget(self._table_ref_geom.prim_path)

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
        #self._shuttle_articulation_controller = self._shuttle.get_articulation_controller()


    async def _add_station(self, station_number):
        station_number =station_number + 1
        self._station_ref_geom = self._world.scene.get_object(f"Station_{station_number}")
        self._station_ref_geom.set_world_pose(position=self.station_info[station_number]['position'],
                                                orientation=self.station_info[station_number]['orientation'])
        self._station_ref_geom.set_default_state(position=self.station_info[station_number]['position'],
                                                orientation=self.station_info[station_number]['orientation'])

        self._asset_ref_geom = self._world.scene.get_object(f"station_{station_number}_asset_ref_geom")
        self._asset_ref_geom.set_local_scale(np.array([self.station_info[station_number]['scale']]))
        self._asset_ref_geom.set_world_pose(position= self.station_info[station_number]['position'])
        self._asset_ref_geom.set_default_state(position=self.station_info[station_number]['position'])
        self._asset_ref_geom.set_collision_approximation("none")


    async def on_sim_control_event_async(self):
        world = self.get_world()
        world.add_physics_callback("sim_step", self.sim_xbots_movement)
        await world.play_async()
        return

    # def on_pmc_connection_event(self):
    #     self.connect_pmc()
    #     return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return




    def sim_xbots_movement(self, step_size):
        #print("step_size: ", step_size)
        #print(self.translate)

        #stage = omni.usd.get_context().get_stage()

        max_speed = 3.0 # m/s
        max_accel = 10.0 # m/s^2

        for shuttle_number in range(1):
        # for shuttle_number in range(self._number_shuttles):
            prim = self.prim_dict["prim_{}".format(shuttle_number + 1)]

            current_pos = prim.GetAttribute('xformOp:translate').Get()

            # Move cube to the right
            if (self._target[1] + 0.1) < current_pos[0]:
                prim.GetAttribute('xformOp:translate').Set((current_pos)-(step_size*max_speed, 0.0, 0.0))
                continue
            # Move cube to the left
            elif (self._target[1] - 0.1) > current_pos[0]:
                prim.GetAttribute('xformOp:translate').Set((current_pos)+(step_size*max_speed, 0.0, 0.0))
                continue

            #current_pos = prim.GetAttribute('xformOp:translate').Get()

            # Move cube up
            if (self._target[0] + 0.1) > current_pos[1]:
                prim.GetAttribute('xformOp:translate').Set((current_pos)+(0.0, step_size*max_speed, 0.0))
                continue
            # Move cube down
            elif (self._target[0] - 0.1) < current_pos[1]:
                prim.GetAttribute('xformOp:translate').Set((current_pos)-(0.0, step_size*max_speed, 0.0))
                continue


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

        xbot_ids = [1, 2, 3, 4, 5, 6, 7, 8]

        # Only update the Xbots if at least 2 seconds have passed since the last update
        if time.time() - self.last_update_time >= 2:

            #print(bot.get_xbot_status(xbot_id=xid).xbot_state)

            xbot_list = bot.get_all_xbot_info(1)
            xbot_positions = [(xbot.x_pos, xbot.y_pos, xbot.z_pos,
                            xbot.rx_pos, xbot.ry_pos, xbot.rz_pos,
                            xbot.xbot_state) for xbot in xbot_list]

            # Don't send commands while the xbots are moving
            if all(xbot_state[6] == pmc_types.XbotState.XBOT_IDLE for xbot_state in xbot_positions):
                # Get random unique targets for each shuttle
                targets_x, targets_y = self.create_random_coordinates(self._number_shuttles)
                print("target_x ", targets_x)
                print("target_y ", targets_y)
                bot.auto_driving_motion_si(8, xbot_ids=xbot_ids, targets_x=targets_x, targets_y=targets_y)
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
                x = random.randint(0, 6) * 0.12 + 0.06
                y = random.randint(0, 8) * 0.12 + 0.06
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

    def connect_pmc(self):

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




############################################################################################################

    def print_cube_info(self, step_size):

        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        # will be shown on terminal
        print("Cube position is : " + str(position))
        print("Cube's orientation is : " + str(orientation))
        print("Cube's linear velocity is : " + str(linear_velocity))

    def _change_property(self, prim_path: str, attribute_name:str, value:float):
        usd_path = Sdf.Path(prim_path + "." + attribute_name)
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=usd_path,
            value=value,
            prev=self._get_property(prim_path, attribute_name),
        )

    def _get_property(self, prim_path: str, attribute: str):
        prim = self.stage.GetPrimAtPath(prim_path)
        prim_property = prim.GetAttribute(attribute)
        return prim_property.Get()
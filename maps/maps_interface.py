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
import carb

from pmclib import system_commands as sys   # PMC System related commands
from pmclib import xbot_commands as bot     # PMC Mover related commands
from pmclib import pmc_types                # PMC API Types
import time



class AcoposInterface(BaseInterface):
    def __init__(
        self,
    )

    self.prim_dict = 



     # Read shuttles position and orientation from physical setup
    def read_xbots_positions(self, step_size):

        # Read info for every shuttle
        xbot_list = bot.get_all_xbot_info(1)
        xbot_positions = [(xbot.x_pos, xbot.y_pos, xbot.z_pos, 
                           xbot.rx_pos, xbot.ry_pos, xbot.rz_pos) for xbot in xbot_list]

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

        prim = self.prim_dict["prim_{}".format(6)]

        # Set position of shuttle
        target = prim.GetAttribute('xformOp:translate').Get()
        print(target)

        bot.linear_motion_si(6,target[0], target[1], 0.5, 10)
        

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
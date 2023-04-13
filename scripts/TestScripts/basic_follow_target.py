# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import cuboid
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
import numpy as np
import os
import argparse

#TODO: Fill in tutorial directory with absolute path to this file
TUTORIAL_DIRECTORY = "E:/Stuff/Faculta/P10/Assets/P10-MAP"

rmp_config_dir = os.path.join(TUTORIAL_DIRECTORY,"Assets/kr3r540")

parser = argparse.ArgumentParser()
parser.add_argument("--urdf_path",type=str,default="kr3r540.urdf")
parser.add_argument("--rmpflow_config_path",type=str,default="kr3r540_rmpflow_config.yaml")
parser.add_argument("--end_effector_frame_name",type=str,default="gripper_base_link")
args = parser.parse_args()

open_stage(usd_path=os.path.join(rmp_config_dir,"kr3r540.usd"))

my_world = World(stage_units_in_meters=0.01)

robot = my_world.scene.add(Robot(prim_path="/kr3r540", name="Kr3r540"))

#Initialize an RmpFlow object
rmpflow = RmpFlow(
    robot_description_path = os.path.join(rmp_config_dir,"kr3_description_v1.0.yaml"),
    urdf_path = os.path.join(rmp_config_dir,args.urdf_path),
    rmpflow_config_path = os.path.join(rmp_config_dir,args.rmpflow_config_path),
    end_effector_frame_name = args.end_effector_frame_name, #This frame name must be present in the URDF
    maximum_substep_size = .0034
)

#Uncomment this line to visualize the collision spheres in the robot_description YAML file
#rmpflow.visualize_collision_spheres()

physics_dt = 1/60.
articulation_rmpflow = ArticulationMotionPolicy(robot,rmpflow,physics_dt)

articulation_controller = robot.get_articulation_controller()

#Make a target to follow
target_cube = cuboid.VisualCuboid("/World/target",position = np.array([0.,0,1.5]),color=np.array([1.,0,0]),size = .1)

my_world.reset()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()

        #Set rmpflow target to be the current position of the target cube.
        rmpflow.set_end_effector_target(
            target_position=target_cube.get_world_pose()[0],
            #target_orientation=target_cube.get_world_pose()[1]
        )

        actions = articulation_rmpflow.get_next_articulation_action()
        articulation_controller.apply_action(actions)

simulation_app.close()

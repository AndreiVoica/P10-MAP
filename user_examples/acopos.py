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
from pxr import Sdf, UsdPhysics

# Can be used to create a new cube or to point to an already existing cube in stage.
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

from omni.isaac.core.utils.nucleus import get_assets_root_path, get_server_path
from omni.isaac.core.utils.stage import add_reference_to_stage, open_stage
from omni.isaac.core.robots import Robot
import carb

class Acopos(BaseSample):
    def __init__(self) -> None:
        super().__init__()

        # SCENE GEOMETRY
        # env (group) spacing:
        self._env_spacing = 2.0

        # Lab Setup:
        self._lab_setup_position = np.array([1.0450, 1.285, 0.0])  # Gf.Vec3f(0.5, 0.0, 0.0)
        setup_orientation = np.pi/2
        self._lab_setup_orientation = np.array([np.cos(setup_orientation/2), 0, 0, np.sin(setup_orientation/2)])
        self._lab_setup_scale = 1.0

        # Shuttle 
        self._shuttle_position = np.array([0.06, 0.06, 1.07])  # Gf.Vec3f(0.5, 0.0, 0.0)
        self._platform_limits = np.array([0.0, 0.0, 0.832, 0.596]) # x_min, y_min, x_max, y_max
        self._target = np.array([0.0, 0.52])
        self._shuttle_scale = 0.01

        self.translate = 0

        # setup asset paths:
        self.asset_folder = "omniverse://localhost/Projects/MAPs-AAU/Assets/"
        self.asset_paths = {
            "table": self.asset_folder + "table.usd",
            "kr3": self.asset_folder + "kr3r540/kr3r540_v3/kr3r540_v3/.usd",
            "kr4": self.asset_folder + "kr4r600/kr4r600_v1/kr4r600_v1/.usd",
            "shuttle": self.asset_folder + "120x120x10/acopos_shuttle_120.usd",
            "lab_setup": self.asset_folder + "Lab_setup_v2.usd"
        }
        return

    # This function is called to setup the assets in the scene for the first time
    # Class variables should not be assigned here, since this function is not called
    # after a hot-reload, its only called to load the world starting from an EMPTY stage
    def setup_scene(self):
        # A world is defined in the AAUSample, can be accessed everywhere EXCEPT __init__

        world = self.get_world()
        world = World.instance()
        world.scene.add_default_ground_plane() # adds a default ground plane to the scene

        add_reference_to_stage(usd_path=self.asset_paths["lab_setup"], prim_path="/World")
        world.scene.add(GeometryPrim(prim_path="/World", name=f"lab_setup_ref_geom", collision=True))


        # add_table_asset
        # add_reference_to_stage(usd_path=self.asset_paths["table"], prim_path="/World/env/table")
        # world.scene.add(GeometryPrim(prim_path="/World/env/table", name=f"table_ref_geom", collision=True))
        
        # #add_shuttle_asset
        add_reference_to_stage(usd_path=self.asset_paths["shuttle"], prim_path="/World/LabSetup/shuttle")
        world.scene.add(GeometryPrim(prim_path="/World/LabSetup/shuttle", name=f"shuttle_ref_geom", collision=True))
       
        return

    # Here we assign the class's variables this function is called after load button is pressed 
    # regardless starting from an empty stage or not this is called after setup_scene and after 
    # one physics time step to propagate appropriate physics handles which are needed to retrieve
    # many physical properties of the different objects
    async def setup_post_load(self):
        self._world = self.get_world()

        self._world.scene.enable_bounding_boxes_computations()
        await self._add_lab_setup()
        await self._add_shuttle()

        self._world.add_physics_callback("sim_step", callback_fn=self.move_shuttle) #callback names have to be unique
        return

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

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
    
    async def _add_table(self):
        ##table
        self._table_ref_geom = self._world.scene.get_object(f"table_ref_geom")
        self._table_ref_geom.set_local_scale(np.array([self._table_scale]))
        self._table_ref_geom.set_world_pose(position=self._table_position,
                                            orientation=self._table_orientation)
        self._table_ref_geom.set_default_state(position=self._table_position, 
                                               orientation=self._table_orientation)
        lb = self._world.scene.compute_object_AABB(name=f"table_ref_geom")
        zmin = lb[0][2]
        zmax = lb[1][2]
        self._table_position[2] = -zmin
        self._table_height = zmax
        self._table_ref_geom.set_collision_approximation("none")
        #self._convexIncludeRel.AddTarget(self._table_ref_geom.prim_path)

    async def _add_lab_setup(self):
        ##lab setup
        self._lab_setup_ref_geom = self._world.scene.get_object(f"lab_setup_ref_geom")
        self._lab_setup_ref_geom.set_local_scale(np.array([self._lab_setup_scale]))
        self._lab_setup_ref_geom.set_world_pose(position=self._lab_setup_position, 
                                                orientation=self._lab_setup_orientation)
        self._lab_setup_ref_geom.set_default_state(position=self._lab_setup_position,
                                                orientation=self._lab_setup_orientation)
        lb = self._world.scene.compute_object_AABB(name=f"lab_setup_ref_geom")
        zmin = lb[0][2]
        zmax = lb[1][2]
        self._lab_setup_position[2] = -zmin
        self._lab_setup_height = zmax
        self._lab_setup_ref_geom.set_collision_approximation("none")
        #self._convexIncludeRel.AddTarget(self._table_ref_geom.prim_path)

    async def _add_shuttle(self):
        ##shuttle
        self._shuttle_ref_geom = self._world.scene.get_object(f"shuttle_ref_geom")
        self._shuttle_ref_geom.set_local_scale(np.array([self._shuttle_scale]))
        self._shuttle_ref_geom.set_world_pose(position=self._shuttle_position)
        self._shuttle_ref_geom.set_default_state(position=self._shuttle_position)
        lb = self._world.scene.compute_object_AABB(name=f"shuttle_ref_geom")
        zmin = lb[0][2]
        zmax = lb[1][2]
        self._shuttle_position[2] = -zmin
        self._shuttle_height = zmax
        self._shuttle_ref_geom.set_collision_approximation("none")
        #self._convexIncludeRel.AddTarget(self._shuttle_ref_geom.prim_path)

    def print_cube_info(self, step_size):
        
        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        # will be shown on terminal
        print("Cube position is : " + str(position))
        print("Cube's orientation is : " + str(orientation))
        print("Cube's linear velocity is : " + str(linear_velocity))

    def move_shuttle(self, step_size):
        #print("step_size: ", step_size)
        print(self.translate)
        
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/World/LabSetup/shuttle")

        current_pos = prim.GetAttribute('xformOp:translate').Get()

        # Move cube to the right
        if self._target[0] < (current_pos[0]-self._shuttle_position[0]):
            prim.GetAttribute('xformOp:translate').Set((current_pos)+(0.01, 0.0, 0.0))
        # Move cube to the left
        elif self._target[0] > (current_pos[0]-self._shuttle_position[0]):
            prim.GetAttribute('xformOp:translate').Set((current_pos)-(0.01, 0.0, 0.0))
        
        current_pos = prim.GetAttribute('xformOp:translate').Get()

        # Move cube up
        if self._target[1] > (current_pos[1]-self._shuttle_position[1]):
            prim.GetAttribute('xformOp:translate').Set((current_pos)+(0.0, 0.01, 0.0))
        # Move cube down
        elif self._target[1] < (current_pos[1]-self._shuttle_position[1]):
            prim.GetAttribute('xformOp:translate').Set((current_pos)-(0.0, 0.01, 0.0))

        # current_pos = prim.GetAttribute('xformOp:translate').Get()
        
        
        #prim.GetAttribute('xformOp:translate').Set((current_pos)+(self.translate, self.translate, 0.01))

        # position, orientation = self._cube.get_world_pose()
        # linear_velocity = self._cube.get_linear_velocity()
        # # will be shown on terminal
        # print("Cube position is : " + str(position))
        # print("Cube's orientation is : " + str(orientation))
        # print("Cube's linear velocity is : " + str(linear_velocity))


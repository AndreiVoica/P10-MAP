from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
from omni.isaac.dynamic_control import _dynamic_control


dc = _dynamic_control.acquire_dynamic_control_interface()
articulation = dc.get_articulation("/World/kr3")
dc.wake_up_articulation(articulation)
dof_ptr = dc.find_articulation_dof(articulation, "joint_a4")
dc.set_dof_position_target(dof_ptr, 0.1)    
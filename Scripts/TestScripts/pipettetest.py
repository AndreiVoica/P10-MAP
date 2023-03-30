from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
from omni.isaac.dynamic_control import _dynamic_control

def home():
    dc = _dynamic_control.acquire_dynamic_control_interface()
    joint1 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint1)
    dof_ptr1 = dc.find_articulation_dof(joint1, "joint_a1")
    dc.set_dof_position_target(dof_ptr1, 0)

    joint2 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint2)
    dof_ptr2 = dc.find_articulation_dof(joint2, "joint_a2")
    dc.set_dof_position_target(dof_ptr2, 0)

    joint3 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint3)
    dof_ptr3 = dc.find_articulation_dof(joint3, "joint_a3")
    dc.set_dof_position_target(dof_ptr3, 0)

    joint4 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint4)
    dof_ptr4 = dc.find_articulation_dof(joint4, "joint_a4")
    dc.set_dof_position_target(dof_ptr4, 0)

    joint5 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint5)
    dof_ptr5 = dc.find_articulation_dof(joint5, "joint_a5")
    dc.set_dof_position_target(dof_ptr5, 0)

    joint6 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint6)
    dof_ptr6 = dc.find_articulation_dof(joint1, "joint_a6")
    dc.set_dof_position_target(dof_ptr6, 0)

def position1():
    dc = _dynamic_control.acquire_dynamic_control_interface()
    joint1 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint1)
    dof_ptr1 = dc.find_articulation_dof(joint1, "joint_a1")
    dc.set_dof_position_target(dof_ptr1, 0)

    joint2 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint2)
    dof_ptr2 = dc.find_articulation_dof(joint2, "joint_a2")
    dc.set_dof_position_target(dof_ptr2, -0.33)

    joint3 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint3)
    dof_ptr3 = dc.find_articulation_dof(joint3, "joint_a3")
    dc.set_dof_position_target(dof_ptr3, 0.342)

    joint4 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint4)
    dof_ptr4 = dc.find_articulation_dof(joint4, "joint_a4")
    dc.set_dof_position_target(dof_ptr4, 0)

    joint5 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint5)
    dof_ptr5 = dc.find_articulation_dof(joint5, "joint_a5")
    dc.set_dof_position_target(dof_ptr5, 0)

    joint6 = dc.get_articulation("/World/kr3")
    dc.wake_up_articulation(joint6)
    dof_ptr6 = dc.find_articulation_dof(joint1, "joint_a6")
    dc.set_dof_position_target(dof_ptr6, 0)


home()
position1()
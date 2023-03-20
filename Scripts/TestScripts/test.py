from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
from omni.isaac.dynamic_control import _dynamic_control

dc = _dynamic_control.acquire_dynamic_control_interface()
articulation1 = dc.get_articulation("/World/kr3_01")
# Call this each frame of simulation step if the state of the articulation is changing.
dc.wake_up_articulation(articulation1)
joint_angles1 = [np.random.rand(9) * 2 - 1]
dc.set_articulation_dof_position_targets(articulation1, joint_angles1)

dc = _dynamic_control.acquire_dynamic_control_interface()
articulation2 = dc.get_articulation("/World/kr3_02")
# Call this each frame of simulation step if the state of the articulation is changing.
dc.wake_up_articulation(articulation2)
joint_angles2 = [np.random.rand(9) * 2 - 1]
dc.set_articulation_dof_position_targets(articulation2, joint_angles2)

dc = _dynamic_control.acquire_dynamic_control_interface()
articulation3 = dc.get_articulation("/World/kr3_03")
# Call this each frame of simulation step if the state of the articulation is changing.
dc.wake_up_articulation(articulation3)
joint_angles3 = [np.random.rand(9) * 2 - 1]
dc.set_articulation_dof_position_targets(articulation3, joint_angles3)

dc = _dynamic_control.acquire_dynamic_control_interface()
articulation4 = dc.get_articulation("/World/kr3_04")
# Call this each frame of simulation step if the state of the articulation is changing.
dc.wake_up_articulation(articulation4)
joint_angles4 = [np.random.rand(9) * 2 - 1]
dc.set_articulation_dof_position_targets(articulation4, joint_angles4)

dc = _dynamic_control.acquire_dynamic_control_interface()
articulation5 = dc.get_articulation("/World/kr4")
# Call this each frame of simulation step if the state of the articulation is changing.
dc.wake_up_articulation(articulation5)
joint_angles5 = [np.random.rand(9) * 2 - 1]
dc.set_articulation_dof_position_targets(articulation5, joint_angles5)

'''
dc = _dynamic_control.acquire_dynamic_control_interface()
articulation1 = dc.get_articulation("/World/kr3_01")
dc.wake_up_articulation(articulation1)
dof_ptr1 = dc.find_articulation_dof(articulation1, "joint_a2")
dc.set_dof_position_target(dof_ptr1, -1)

articulation2 = dc.get_articulation("/World/kr3_02")
dc.wake_up_articulation(articulation2)
dof_ptr2 = dc.find_articulation_dof(articulation2, "joint_a2")
dc.set_dof_position_target(dof_ptr2, -1)
'''
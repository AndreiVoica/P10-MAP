from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
import numpy as np
from tasks.pick_place import PickPlace

my_world = World(stage_units_in_meters=1.0)


target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.0515 / 2.0
my_task = PickPlace(name="denso_pick_place", target_position=target_position)
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("denso_pick_place").get_params()
denso_name = task_params["robot_name"]["value"]
my_denso = my_world.scene.get_object(denso_name)
articulation_controller = my_denso.get_articulation_controller()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        observations = my_world.get_observations()
        print(observations)
simulation_app.close()
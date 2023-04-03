from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("kuka_kr3r540", package_name="kuka_kr3_config_test3_nogripper").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)

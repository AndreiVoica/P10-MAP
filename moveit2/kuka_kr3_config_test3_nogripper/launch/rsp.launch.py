from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("kuka_kr3r540", package_name="kuka_kr3_config_test3_nogripper").to_moveit_configs()
    return generate_rsp_launch(moveit_config)

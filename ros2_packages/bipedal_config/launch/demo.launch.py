from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.actions import Node
from launch import LaunchDescription



def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("bipedal_robot", package_name="bipedal_config").to_moveit_configs()


    # MoveIt 기본 데모 런치 구성 생성
    pkg_share = Path(get_package_share_directory("bipedal_config"))

    moveit_config = (
        MoveItConfigsBuilder("bipedal_robot", package_name="bipedal_config")
        .robot_description(file_path=str(pkg_share / "config" / "bipedal_robot.urdf.xacro"))
        .robot_description_semantic(file_path=str(pkg_share / "config" / "bipedal_robot.srdf"))
        .to_moveit_configs()
    )

    demo_launch = generate_demo_launch(moveit_config)  # LaunchDescription 반환
    recorder_node = Node(
        package="python_tools",                 
        executable="record_on_execute",
        name="record_on_execute",
        output="screen",
        parameters=[
            {'controller_name': 'total_controller'},
            {'package_name': 'bipedal_workspace'},
        ]
    )

    return LaunchDescription([
        *demo_launch.entities,  
        recorder_node,
    ])
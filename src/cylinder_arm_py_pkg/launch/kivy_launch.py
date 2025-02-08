from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

package_name = 'cylinder_arm_py_pkg'

def get_robot_config(context: LaunchContext, config_file: LaunchConfiguration):
    share_dir = get_package_share_directory(package_name)
    gui_config_file = os.path.join(share_dir, 'resources', context.perform_substitution(config_file))
    return [ExecuteProcess(
            cmd=['ros2', 'run', package_name, 'kivy_app', '--config-file', gui_config_file],
            output='screen'
        )]

def generate_launch_description():
    config_file = LaunchConfiguration(
            'config_file',
            default="robot_config.json",
    )
    return LaunchDescription([
        Node(
            package=package_name,
            executable="hardware_service",
            name="hw_service",
            parameters=[{'NO_HARDWARE':True}]
        ),
        OpaqueFunction(function=get_robot_config, args=[config_file])
    ])
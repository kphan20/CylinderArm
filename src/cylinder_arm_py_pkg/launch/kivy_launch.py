from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

package_name = 'cylinder_arm_py_pkg'

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', package_name, 'kivy_app'],
            output='screen'
        ),
        Node(
            package=package_name,
            executable="hardware_service",
            name="hw_service",
            parameters=[{'NO_HARDWARE':True}]
        )
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_name = 'px4_trajectory_tracking'
    pkg_share = get_package_share_directory(package_name)

    rviz_config_path = os.path.join(pkg_share, 'config', 'drone_view.rviz')

    return LaunchDescription([
        # ------------------------------------
        # MicroXRCE-DDS Agent (the communication bridge)
        # ------------------------------------
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'
        ),

        # ------------------------------------
        # The Control Node
        # ------------------------------------
        Node(
            package=package_name,
            executable='start_agent',
            name='offboard_agent',
            output='screen',
            emulate_tty=True
        ),

        # ------------------------------------
        # RViz2
        # ------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),

        # ------------------------------------
        # PlotJuggler
        # ------------------------------------
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            arguments=['--nosplash'],
            output='screen'
        )
    ])
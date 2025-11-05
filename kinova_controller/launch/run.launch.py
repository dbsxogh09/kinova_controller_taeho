import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Create the launch configuration variables
    autostart_param = DeclareLaunchArgument(
        name='autostart',
        default_value='True',
        description='Automatically start lifecycle nodes')
    priority_param = DeclareLaunchArgument(
        name='priority',
        default_value='95',
        description='Set process priority')
    cpu_affinity_param = DeclareLaunchArgument(
        name='cpu-affinity',
        default_value='1',
        description='Set process CPU affinity')
    with_lock_memory_param = DeclareLaunchArgument(
        name='lock-memory',
        default_value='True',
        description='Lock the process memory')
    lock_memory_size_param = DeclareLaunchArgument(
        name='lock-memory-size',
        default_value='500',
        description='Set lock memory size in MB')
    config_child_threads_param = DeclareLaunchArgument(
        name='config-child-threads',
        default_value='True',
        description='Configure process child threads (typically DDS threads)')

    # Node definitions
    kinova_controller_runner = Node(
        package='kinova_controller',
        executable='run',
        output='screen',
        # parameters=[param_file],
        arguments=[
           '--autostart', LaunchConfiguration('autostart'),
        #    '--priority', LaunchConfiguration('priority'),
        #    '--cpu-affinity', LaunchConfiguration('cpu-affinity'),
        #    '--lock-memory', LaunchConfiguration('lock-memory'),
        #    '--lock-memory-size', LaunchConfiguration('lock-memory-size'),
        #    '--config-child-threads', LaunchConfiguration('config-child-threads')
           ]
    )

    ld = LaunchDescription()

    ld.add_action(autostart_param)
    ld.add_action(priority_param)
    ld.add_action(cpu_affinity_param)
    ld.add_action(with_lock_memory_param)
    ld.add_action(lock_memory_size_param)
    ld.add_action(config_child_threads_param)
    ld.add_action(kinova_controller_runner)

    return ld
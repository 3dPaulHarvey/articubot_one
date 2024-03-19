import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name='articubot_one' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )


    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    # Path to the controller configuration YAML file
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')


    
    # Launch the controller_manager and load the controllers
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params_file],
    )

    delayed_controller_manager_node = TimerAction(period=3.0, actions=[controller_manager_node])

    load_diff_cont = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    delayed_load_diff_cont = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[load_diff_cont],
        )
    )

    load_joint_broad = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    delayed_load_joint_broad = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[load_joint_broad],
        )
    )

# ####################
#     load_and_start_controller2 = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_broad', '--controller-manager', '/controller_manager'],
#     )
# ###############
    
    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager_node,
        delayed_load_diff_cont,
        delayed_load_joint_broad,

    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name='articubot_one' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    #Set a higher frame rate for gazebo 
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')


    load_diff_cont = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    load_joint_broad = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )


# ####################
#         # Path to the controller configuration YAML file
#     controller_config = os.path.join(
#         get_package_share_directory(package_name),
#         'config',
#         'my_controllers.yaml'
#     )

#     # Launch the controller_manager and load the controllers
#     controller_manager_node = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[{'use_sim_time': True},
#                     controller_config],
#         output='screen',
#     )


#     load_and_start_controller2 = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_broad', '--controller-manager', '/controller_manager'],
#     )


# ###############




    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        # controller_manager_node,
        load_diff_cont,
        load_joint_broad,
    ])

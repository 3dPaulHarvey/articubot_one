from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():


    # joy_config_file = os.path.join(
    #     get_package_share_directory('articubot_one'),
    #     'bringup',
    #     'config',
    #     'joystick_params.yaml'
    # )
    joy_config_file = os.path.join(get_package_share_directory('articubot_one'),'config','joystick_params.yaml')


    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_config_file],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config_file],
        ),
        Node(
            package='articubot_one',
            executable='joy_to_twist_stamped',
            name='joy_to_twist_stamped_node',
            output='screen',
        )
    ])

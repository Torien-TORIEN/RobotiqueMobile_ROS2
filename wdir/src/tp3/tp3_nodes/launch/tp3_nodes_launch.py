from launch import LaunchDescription
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import os
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    description_dir = get_package_share_directory('tp3_description')
    launch_dir = os.path.join(description_dir, 'launch')

    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'tp3_description_launch.py')
        ),
    )

    # creating the teleop node : ros2 run tp1 my_teleop.py
    escaperoom_node = Node( package='tp3_nodes',
                        executable='escape_room.py',
                        name='escape_room',
                        remappings=[ # rename /turtle1/cmd_vel to /turtlesim1/caroline/cmd_vel
                        ],
                        prefix=["xterm -e"], # start the node into an xterm terminal
                    )

    commands = [
        gazebo_launch_cmd,
        escaperoom_node,
        
    ]

    #Equivalent de add_action
    return LaunchDescription(commands)

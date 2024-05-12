# These import statements pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

# The launch description itself begins here:
def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = FindPackageShare(package='tp3_description').find('tp3_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/base.rviz')
    urdf_path = os.path.join(
        pkg_share,
       'urdf/tp3_robot.urdf'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
    )
    state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(["cat ", urdf_path]),
        }]
    )

    joint_publisher_cmd = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )


    ld.add_action(start_rviz_cmd)
    ld.add_action(state_publisher_cmd)
    ld.add_action(joint_publisher_cmd)

    return ld

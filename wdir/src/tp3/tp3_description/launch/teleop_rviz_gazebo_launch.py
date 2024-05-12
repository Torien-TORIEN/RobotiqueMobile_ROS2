from launch import LaunchDescription
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import os
from launch.substitutions import Command

def generate_launch_description():
    
    gazebo_world_path = PathJoinSubstitution(
        [FindPackageShare('tp3_description'), 'gazebo', 'my_first_world.sdf']
    )

    pkg_share = FindPackageShare(package='tp3_description').find('tp3_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/lidar_gazebo.rviz')
    urdf_path = os.path.join(
        pkg_share,
       'urdf/tp3_robot.urdf'
    )

    #Commande : ign gazebo tep3_description my_first_world.sdf -r
    gazebo_cmd = ExecuteProcess(
        cmd = [[
            FindExecutable(name="ign"),
            " gazebo ",
            gazebo_world_path,
            " -r",
        ]],
        shell = True
    )


    #ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
    brige_node_cmd_vel=Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
            prefix=["xterm -e"],
        )
    
    #ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan
    brige_node_lidar=Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=['/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
            prefix=["xterm -e"],
        )

    # creating the teleop node : ros2 run tp1 my_teleop.py
    teleop_node = Node( package='tp1',
                        namespace='turtlesim1',
                        executable='my_teleop.py',
                        name='sim_key',
                        remappings=[ # rename /turtle1/cmd_vel to /turtlesim1/caroline/cmd_vel
                                    ('/turtle1/cmd_vel', '/cmd_vel'),],
                        prefix=["xterm -e"], # start the node into an xterm terminal
                    )

    #Launch rviz
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

    commands = [
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=FindPackageShare('tp3_description')),
        gazebo_cmd,
        teleop_node,
        brige_node_cmd_vel,
        brige_node_lidar,
        start_rviz_cmd,
        state_publisher_cmd,
        joint_publisher_cmd

    ]

    #Equivalent de add_action
    return LaunchDescription(commands)
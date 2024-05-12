from launch import LaunchDescription
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    
    gazebo_world_path = PathJoinSubstitution(
        [FindPackageShare('tp3_description'), 'gazebo', 'my_first_world.sdf']
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
    brige_node=Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
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

    commands = [
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=FindPackageShare('tp3_description')),
        gazebo_cmd,
        teleop_node,
        brige_node
    ]

    #Equivalent de add_action
    return LaunchDescription(commands)

from launch import LaunchDescription
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    
    gazebo_world_path = PathJoinSubstitution(
        [FindPackageShare('tp3_description'), 'gazebo', 'my_first_world.sdf']
    )

    gazebo_cmd = ExecuteProcess(
        cmd = [[
            FindExecutable(name="ign"),
            " gazebo ",
            gazebo_world_path,
        ]],
        shell = True
    )

    commands = [
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=FindPackageShare('tp3_description')),
        gazebo_cmd,
    ]

    return LaunchDescription(commands)

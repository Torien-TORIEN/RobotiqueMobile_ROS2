# These import statements pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

# The launch description itself begins here:
def generate_launch_description():
    ld = LaunchDescription()
    
    # creating the node turtlesim 
    turtle_node = Node( package='turtlesim',
                        namespace='turtlesim1',
                        executable='turtlesim_node',
                        name='sim1'
                    )

    # creating the node my_teleop --> caroline
    teleop_node = Node( package='tp1',
                        namespace='turtlesim1',
                        executable='my_teleop.py',
                        name='sim_key',
                        remappings=[ # rename /turtle1/cmd_vel to /turtlesim1/caroline/cmd_vel
                                    ('/turtle1/cmd_vel', '/turtlesim1/caroline/cmd_vel'),],
                        prefix=["xterm -e"], # start the node into an xterm terminal
                    )
    
    # creating the node my_teleop --> turtle1
    teleop_node_2 = Node( package='tp1',
                        namespace='turtlesim1',
                        executable='my_teleop.py',
                        name='sim_key',
                        remappings=[ # rename /turtle1/cmd_vel to /turtlesim1/caroline/cmd_vel
                                    ('/turtle1/cmd_vel', '/turtlesim1/turtle1/cmd_vel'),],
                        prefix=["xterm -e"], # start the node into an xterm terminal
                    )

    # creating a process to execute after starting the node -->caroline
    ep = ExecuteProcess(
            cmd = [[
                FindExecutable(name="ros2"),
                " service call ",
                "/turtlesim1/spawn ",
                "turtlesim/srv/Spawn ",
                '"{x: 2, y: 2, theta: 0.2, name: \'caroline\'}"',
            ]],
            shell = True
        )
    # add the node
    ld.add_action(turtle_node)

    # add the node
    ld.add_action(teleop_node)

    # add the node
    ld.add_action(teleop_node_2)

    # add the process
    ld.add_action(ep)

    return ld

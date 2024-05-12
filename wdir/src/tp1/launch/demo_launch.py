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
    # creating the node Follo Path Action server 
    pathActionServer_node = Node( package='tp1',
                        namespace='turtlesim1',
                        executable='follow_path.py',
                        name='sim_order',
                        remappings=[ ('/turtle1/pose', '/turtlesim1/turtle1/pose'),
                                    ('/turtle1/cmd_vel', '/turtlesim1/turtle1/cmd_vel'),
                                    ('/path', '/turtlesim1/path'),
                        ],
                        prefix=["xterm -e"], # start the node into an xterm terminal
                    )


    # creating a process to execute after starting the node -->caroline
    
    
    ep = ExecuteProcess(
            cmd = [
                [
                    FindExecutable(name="ros2"),
                    " action send_goal ",
                    "/turtlesim1/path ",
                    " tp1/action/Path ",
                    '"{ path: [{x: 5.0,y: 5.0,z: 0.0},{x: 6.0,y: 5.0,z: 0.0},{x: 7.0,y: 4.0,z: 0.0},{x: 7.0,y: 2.0,z: 0.0},{x: 6.0,y: 1.0,z: 0.0},{x: 1.0,y: 1.0,z: 0.0},{x: 1.0,y: 5.0,z: 0.0},{x: 3.0,y: 5.0,z: 0.0},{x: 5.0,y: 7.0,z: 0.0},{x: 6.0,y: 7.0,z: 0.0},{x: 5.0,y: 5.0,z: 0.0}]}" --feedback'
                ]
            ]
            ,
            shell = True,
            prefix=["xterm -e"], # start the node into an xterm terminal
        )
    # add the node
    ld.add_action(turtle_node)

    # add the node
    ld.add_action(pathActionServer_node)

    # add the process
    ld.add_action(ep)

    return ld

#!/bin/python3

# import ROS libraries
import rclpy
from rclpy.node import Node
import sys, tty, termios
import threading
from std_srvs.srv import Empty

# imports the built-in string message type
# it will the type of the published messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Teleop(Node):
    # The MinimalPublisher class inherits from the Node class

    def __init__(self):
        self.publisher_: rclpy.publisher.Publisher = None  # publisher to publish the message
        self.clear_path_service = None

        super().__init__('my_teleop')

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) # type de massg , nom de mon topic , taille de buffer
        self.clear_path_service = self.create_client(Empty, 'clear')  #type de service , nom de service 

        # self.publisher_ = self.create_publisher(String, 'MonTopic', 10) # type de massg , nom de mon topic , taille de buffer

        # log with [INFO] flag
        self.get_logger().info("The commands:\n\t'o' to move forward\n\t'l' to move backward\n\t'k' to turn left\n\t'm' to turn right\n\t'c' to clear the path\n\t'q' to quit")


    

    

    def myLoop(self):

        while True:
            move_cmd = Twist()
            key=getkey()

            if key=="q":
                rclpy.shutdown()
                break
                
            elif key == "c": #clear path
                request = Empty.Request()
                self.clear_path_service.call_async(request)

            elif key == "o": #move forward
                move_cmd.linear.x = 0.5
                move_cmd.angular.z = 0.0
            
            elif key == "l": #move backward
                move_cmd.linear.x = -0.5
                move_cmd.angular.z = 0.0

            elif key == "k": #rotate left
                move_cmd.linear.x = 0.0
                move_cmd.linear.y = 0.0
                move_cmd.angular.z = 0.5

            elif key == "m": #rotate right
                move_cmd.linear.x = 0.0
                move_cmd.linear.y = 0.0
                move_cmd.angular.z = -0.5
            
           

            self.publisher_.publish(move_cmd)  # publish the message


        


def getkey():
    """ To get a keyboad key (only one char)
        without waiting for Enter to be pressed 
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch




def main(args=None):
    # to init the ros interface
    rclpy.init(args=args)

    # creating the node from our custom class
    teleop = Teleop()
    
    # start the node and loop until it closes
    my_thread = threading.Thread(target=teleop.myLoop)  # the target is the function to call when running the thread
    my_thread.start()  # start the thread


    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt as ki:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop.destroy_node()
    # close the ros interface
    #rclpy.shutdown()


if __name__ == '__main__':
    main()

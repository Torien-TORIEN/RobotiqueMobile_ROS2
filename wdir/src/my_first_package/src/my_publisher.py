#!/bin/python3

# import ROS libraries
import rclpy
from rclpy.node import Node

# imports the built-in string message type
# it will the type of the published messages
from std_msgs.msg import String

class MinimalPublisher(Node):
    # The MinimalPublisher class inherits from the Node class

    def __init__(self):

        self.publisher_: rclpy.publisher.Publisher = None  # publisher to publish the message
        self.timer: rclpy.timer.Timer = None               # timer to send the message periodically
        self.i: int = None                                # counter for the index of messages

        # constructor of the class

        # call the upper class constructor (the Node constructor)
        # the str as argument is to define the node name when running (ros2 node list)
        super().__init__('minimal_publisher')

        # create a publisher to publish the messages
        #   String : the type of message (imported from std_msgs.msg)
        #   'myTopic' : the topic name to send the messages
        #   10 : the “queue size” is 10.
        #       Queue size is a required QoS (quality of service) setting that limits the amount of queued messages
        #       if a subscriber is not receiving them fast enough.
        self.publisher_ = self.create_publisher(String, 'myTopic', 10) # type de massg , nom de mon topic , taille de buffer

        # A timer is created with a callback to execute every 0.5 seconds.
        # self.i is a counter used in the callback.
        timer_period = 0.5  # 0.5 second -> 500ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # timer_callback creates a message with the counter value appended,
        # and publishes it to the console with get_logger().info.
        msg = String()
        msg.data = f'Hello World: {self.i}' 
        self.publisher_.publish(msg)  # publish the message
        # log with [INFO] flag
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    # to init the ros interface
    rclpy.init(args=args)

    # creating the node from our custom class
    minimal_publisher = MinimalPublisher()

    # start the node and loop until it closes

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt as ki:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    # close the ros interface
    #rclpy.shutdown()


if __name__ == '__main__':
    main()

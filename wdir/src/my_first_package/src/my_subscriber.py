#!/bin/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):
    
    subscription: rclpy.subscription.Subscription

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String, # the message type
            'myTopic', # the topic name
            self.listener_callback, # function called when getting a message
            10)

    def listener_callback(self, msg:String):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    #rclpy.spin(minimal_subscriber)
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt as ki:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/bin/python3

import time # needed for the sleep

# ROS2 lirabries
import rclpy
from rclpy.action import ActionServer  # needed for the action server
from rclpy.node import Node

# The considered interface for the action
from tp1.action import Path

class PathActionServer(Node):
    """
    Class to define a new node, that will provide the action server
    """

    action_server: rclpy.action.server.ActionServer  # The action server for this node

    def __init__(self):
        super().__init__('path_action_server')
        self.action_server = ActionServer(
            self,
            Path,               # The type of the action
            'path',             # The action name
            self.follow_path)   # The callback function

    def follow_path(self, goal_handle:rclpy.action.server.ServerGoalHandle):
        """
        The callback function, called when receiving a goal for the /path action
        goal_handle: the goal
        """
        self.get_logger().info('Executing goal...')

        # variable to handle the feedback (the messages sent before the end of the action)
        feedback_msg = Path.Feedback()

        # for this example we just loop over all the point of the path
        for point in goal_handle.request.path:
            # for each point, we define the feedback point as the current point
            feedback_msg.reached_point = point
            # we publish the feedback
            goal_handle.publish_feedback(feedback_msg)
            # we wait for 1 second
            time.sleep(1)

        # to indicate that the goal was successful (otherwise it is assumed aborted by default)
        goal_handle.succeed()

        # variable to handle the resulting message
        result = Path.Result()
        # for this example it is just the last point of the path
        result.final_point = goal_handle.request.path[-1]
        self.get_logger().info('...Finished')
        return result


def main(args=None):
    rclpy.init(args=args)
    path_action_server = PathActionServer()
    try:
        rclpy.spin(path_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

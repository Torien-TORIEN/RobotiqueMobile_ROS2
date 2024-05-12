#!/bin/python3



from std_msgs.msg import String
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
import math

# ROS2 lirabries
import rclpy
from rclpy.action import ActionServer  # needed for the action server
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


# The considered interface for the action
from tp1.action import Path

class FollowPathActionServer(Node):
    
    subscription1: rclpy.subscription.Subscription
    subscription2: rclpy.subscription.Subscription
    pose=Pose()

    def __init__(self):
        super().__init__('MyFollowPath')
        self.goal_handle = None
        self.publisher_: rclpy.publisher.Publisher = None  # publisher to publish the message
        


        # creation of the callback groups
        group1 = MutuallyExclusiveCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()
        #pose = Point()

        # two subscribers, each one affected to a callback group
        self.subscription1 = self.create_subscription( Pose,'turtle1/pose', self.getPose_listener_callback1, 1, callback_group=group1)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        #self.subscription2 = self.create_subscription( String,'myTopic', self.listener_callback2, 1, callback_group=group2)

        self.action_server = ActionServer(
            self,
            Path,               # The type of the action
            'path',             # The action name
            self.follow_path,
            callback_group=group2)   # The callback function"""

    # callback for the first subscriber : get pose
    def getPose_listener_callback1(self, pos:Pose):
        self.pose=pos



    def follow_path(self, goal_handle:rclpy.action.server.ServerGoalHandle):
        """
        The callback function, called when receiving a goal for the /path action
        goal_handle: the goal
        """
        t_debut=time.time()
        if self.goal_handle is not None:
            goal_handle.abort()
            return Path.Result()
        self.goal_handle = goal_handle
        self.get_logger().info('Executing goal...')

        # variable to handle the feedback (the messages sent before the end of the action)
        feedback_msg = Path.Feedback()

        # for this example we just loop over all the point of the path
        for point in goal_handle.request.path:
            # for each point, we define the feedback point as the current point
            #self.get_logger().info(f'next Point :{point}')
            move_cmd = Twist()
            pos=self.pose
            n=0
            while abs(pos.x - point.x ) > 0.1 or abs(pos.y - point.y ) > 0.1:
                pas=min(abs(pos.x - point.x ),abs(pos.y - point.y ) )

                #Turn the turtle
                if(point.x < pos.x):
                    alpha=math.atan((point.y-pos.y)/((2*pos.x-point.x)-pos.x))
                    omega=math.pi/2-alpha
                    gamma1=alpha-pos.theta
                    gamma=2*omega+gamma1
                    if gamma >=6: gamma -=2*math.pi

                else :
                    alpha=math.atan((point.y-pos.y)/(point.x-pos.x))
                    gamma=alpha-pos.theta

                

                """if(n==0):
                    move_cmd.linear.x = 0.0
                    move_cmd.linear.y = 0.0
                    move_cmd.angular.z = gamma
                    self.publisher_.publish(move_cmd)  # publish the message
                    n+=1
                    time.sleep(1)
                    continue"""
                
                move_cmd.linear.x = 0.0
                move_cmd.linear.y = 0.0
                move_cmd.angular.z = gamma
                #self.publisher_.publish(move_cmd)  # publish the message"""
                
                #Move the robot
                #self.get_logger().info(f'Position ( x:{pos.x},y:{pos.y},theta:{pos.theta},alpha:{alpha},gamma:{gamma} )')
                #self.get_logger().info(f'time ={time.time()-t_debut}, Ax:{point.x-pos.x},Ay:{point.y-pos.y} )')
                move_cmd.linear.x = 0.09999
                move_cmd.linear.y = 0.0
                #move_cmd.angular.z = 0.0
                self.publisher_.publish(move_cmd)  # publish the message
                
                time.sleep(1.0)

                pos=self.pose

            p=Point(); p.x,p.y=self.pose.x,self.pose.y
            feedback_msg.reached_point = p
            # we publish the feedback
            goal_handle.publish_feedback(feedback_msg)
            # we wait for 1 second
            time.sleep(1)

        # to indicate that the goal was successful (otherwise it is assumed aborted by default)
        goal_handle.succeed()

        # variable to handle the resulting message
        result = Path.Result()
        # for this example it is just the last point of the path
        #result.final_point = goal_handle.request.path[-1]
        result.final_point =p
        self.get_logger().info(f'...Finished in {time.time()-t_debut}s')
        self.goal_handle = None
        return result



def main(args=None):
    rclpy.init(args=args)
    my_node = FollowPathActionServer()
    my_node.get_logger().info("Beginning path_action_server, shut down with CTRL-C")
    # declaration of the multithreaded executor
    executor = MultiThreadedExecutor()
    # adding the node to the executor
    executor.add_node(my_node)
    # spinning the executor
    try:
        executor.spin()
    except KeyboardInterrupt as ki:
        pass
    

    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""ME DRAWN
ros2 action send_goal /path tp1/action/Path "{path:[
{x: 5.544445,y: 5.544445,z: 0.0},
{x: 5.544445,y: 7.544445,z: 0.0},
{x: 6.544445,y: 6.544445,z: 0.0},
{x: 7.544445,y: 7.544445,z: 0.0},
{x: 7.544445,y: 5.544445,z: 0.0},
{x: 9.544445,y: 5.544445,z: 0.0},
{x: 8.544445,y: 5.544445,z: 0.0},
{x: 8.544445,y: 6.544445,z: 0.0},
{x: 9.544445,y: 6.544445,z: 0.0},
{x: 8.544445,y: 6.544445,z: 0.0},
{x: 8.544445,y: 7.544445,z: 0.0},
{x: 9.544445,y: 7.544445,z: 0.0}]}" --feedback
"""

""" U DRAWN
ros2 action send_goal /path tp1/action/Path "{path:[
{x: 5.544445,y: 5.544445,z: 0.0},
{x: 5.544445,y: 6.544445,z: 0.0},
{x: 5.544445,y: 5.544445,z: 0.0},
{x: 6.544445,y: 5.544445,z: 0.0},
{x: 6.544445,y: 6.544445,z: 0.0},
{x: 6.544445,y: 5.544445,z: 0.0}]}" --feedback
"""
""" HEART DRAWN
ros2 action send_goal /path tp1/action/Path "{path:[
{x: 5.544445,y: 5.544445,z: 0.0},
{x: 6.544445,y: 6.544445,z: 0.0},
{x: 7.544445,y: 6.544445,z: 0.0},
{x: 8.544445,y: 5.544445,z: 0.0},
{x: 8.544445,y: 4.544445,z: 0.0},
{x: 8.044445,y: 3.544445,z: 0.0},
{x: 5.544445,y: 1.544445,z: 0.0},
{x: 3.044445,y: 3.544445,z: 0.0},
{x: 2.544445,y: 4.544445,z: 0.0},
{x: 2.544445,y: 5.544445,z: 0.0},
{x: 3.544445,y: 6.544445,z: 0.0},
{x: 4.544445,y: 6.544445,z: 0.0},
{x: 5.544445,y: 5.544445,z: 0.0}]}" --feedback 
"""

#!/bin/python3



from std_msgs.msg import String
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

#Here Libraries
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

#Python Libraries
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

class EscapeRoom(Node):
    
    subscription_lidar: rclpy.subscription.Subscription
    subscription_start: rclpy.subscription.Subscription
    pose=Pose()
    point= Point()

    def __init__(self):
        super().__init__('EscapeRoom')
        self.goal_handle = None
        self.angle=None
        self.isOut=False
        self.publisher_finish: rclpy.publisher.Publisher = None  # publisher to publish the message
        self.pub_marker: rclpy.publisher.Publisher = None  # publisher to publish the message
        self.publisher_cmd: rclpy.publisher.Publisher = None  # publisher to publish the message
        


        # creation of the callback groups
        group1 = MutuallyExclusiveCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()

        # two subscribers, each one affected to a callback group
        self.subscription_lidar = self.create_subscription( LaserScan,'/lidar', self.getLidar_listener_callback, 1, callback_group=group1)
        self.subscription_start = self.create_subscription( Bool,'/tp3/start', self.escape_room_callback, 1, callback_group=group2)
        self.pub_marker = self.create_publisher(Marker, '/tp3/debug_marker', 10)

        self.publisher_finish = self.create_publisher(Bool, '/tp3/finish', 10)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        




    # callback for the first subscriber : get pose
    def getLidar_listener_callback(self, scan:LaserScan):

        angle = scan.angle_min
        for i, dst in enumerate(scan.ranges): # loop over all the LiDAR measurements
            if dst != float("inf"): # if the measurement detected an obstacle
                #self.get_logger().info(f"{angle=}, distance={scan.ranges[i-1]})")
                time.sleep(0)
            else:
                #self.get_logger().info(f"Door : Angle = {angle}, distance={scan.ranges[i-1]})")
                distance=scan.ranges[i-1]
                self.point.x=distance * math.cos(angle)
                self.point.y=distance * math.sin(angle)
                self.angle=angle
                #self.get_logger().info(f"Point {self.point}")
                if(abs(self.point.y)!=float("inf") and abs(self.point.x)!=float("inf")) :
                    self.create_marker(1,self.point,255.0,0.0,0.0)
                    break
            angle += scan.angle_increment


    def escape_room_callback(self, msg:Bool):

        #Check if my robot is already out
        if abs(self.point.x == float("inf")) and abs(self.point.y == float("inf")):
            self.isOut=True

        #Check if message is True and my robot is inside  
        if msg and not self.isOut :
            orientation=self.find_orientation(self.angle)
            alpha=math.atan(self.point.y/abs(self.point.x))

            self.get_logger().info(f"ROBOT IS ESCAPING ROOM ...")
            move_cmd = Twist()

            # Turning in the right direction and aligning parallel
            self.get_logger().info(f"\t step 1: Aligning parallel to the door ...")

            #To  optimize 
            go_out_oblique=False
            parallel_position_required=False
            first_angle=self.angle

            while True:

                
                if orientation in ['E' ,'W'] :
                    # Reculer ou Avancer direct
                    move_cmd.angular.z = 0.0
                    move_cmd.linear.x = 0.1 #Avancer 
                    step2 = "Move forward"
                    if(self.point.x < 0):
                        move_cmd.linear.x = -0.1 #reculer
                        step2 = "Move backward"
                        
                        
                    #Avancer ou Reculer
                    self.get_logger().info(f"\t step 2: {step2} parallel to the door ...")
                    while True :
                        self.publisher_cmd.publish(move_cmd)
                        if orientation=="E" and abs(self.point.x + 1 ) < 0.2:
                            #self.get_logger().info(f"Stop E")
                            break

                        elif orientation=="W" and abs(self.point.x - 1.0) < 0.2:
                            #self.get_logger().info(f"Stop W :")
                            break
                        
                        
                    # Quitter
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.0
                    self.publisher_cmd.publish(move_cmd)
                    break

                elif orientation in ['N' ,'S'] :
                    #Tourner pour se mettre en paralèlle puis Avancer ou reculer
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.1

                    
                    #if orientation is North and a door is so near from west wall
                    if(orientation=="N" and first_angle > 0.12 ):#0.22
                        parallel_position_required=True
                        #move_cmd.angular.z = -0.1
                    elif orientation == "N" and first_angle<-0.20 and first_angle>-0.36:
                        move_cmd.angular.z = -0.1



                    self.publisher_cmd.publish(move_cmd)
                    #self.get_logger().info(f"Angle= {self.angle} (x,y)=({self.point.x},{self.point.y})")

                    #on est en paralèlle et avancer ou reculer 
                    if( abs(abs(self.angle) + alpha - math.pi/2)< 0.01):
                        move_cmd.linear.x = 0.1
                        move_cmd.angular.z = 0.0
                        step2 = "Move forward"

                        if(self.point.x < 0):
                            move_cmd.linear.x = -0.1
                            step2 = "Move backward"
                        
                        self.get_logger().info(f"\t step 2: {step2} parallel to the door ...")
                        while True :
                            self.publisher_cmd.publish(move_cmd)
                            if orientation=="S" and abs(self.point.x - 1 ) < 0.2: # porte doit etre en x=1
                                break
                            if orientation=="S" and abs(self.point.x - 1 ) < 0.3 and (first_angle -2.96459)<0.00001: # porte doit etre en x=1
                                break
                            elif orientation=="N" and abs(self.point.x + 1 ) < 0.2: # porte doit etre en x= -1
                                break
                        
                        move_cmd.linear.x = 0.0
                        move_cmd.angular.z = 0.0
                        self.publisher_cmd.publish(move_cmd)
                        break

                    if -0.23 < self.angle and  self.angle <-0.19 and not parallel_position_required :#[0 à +- 0.3097]
                        #self.get_logger().info(f"=> {self.angle}")
                        go_out_oblique=True
                        break

            # Turn again if not completed to face the door
            self.get_logger().info(f"\t step 3: Turning to face the door...")
            while True:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z =  0.1 # +0.1

                if(orientation=="E" or (orientation=="N" and not go_out_oblique)):
                    move_cmd.angular.z =  -0.1 

                self.publisher_cmd.publish(move_cmd)
                #self.get_logger().info(f"Angle= {self.angle} (x,y)=({self.point.x},{self.point.y})")
                if -0.25 < self.angle and  self.angle <-0.19 :#[0 à +- 0.3097]
                    break
            
            #Stop turning 
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.publisher_cmd.publish(move_cmd)

            # Go forward to escape the room
            self.get_logger().info(f"\t step 4: Exiting the room ...")
            while  True:
                #self.get_logger().info(f"Angle= {self.angle}")
                move_cmd.linear.x = 1.0
                move_cmd.angular.z = 0.0 #0.03
                """if(self.angle<=-0.3) :
                    move_cmd.angular.z = 0.06
                elif -0.19<= self.angle:
                    move_cmd.angular.z = -0.06
                else:
                    move_cmd.angular.z = 0.0 """
        
                self.publisher_cmd.publish(move_cmd)
                #self.get_logger().info(f"Angle= {self.angle} (x;y)=({self.point.x};{self.point.y})" )
                
                #My robot is almost out
                if abs(abs(self.angle) -math.pi)<0.000005:
                    #self.get_logger().info(f"Angle= {self.angle}")

                    # Wait for 4 secondes time my robot will be outside completely
                    import time
                    time.sleep(2)
                    break

            #Stop going forward
            self.get_logger().info(f" ROBOT ESCAPED THE ROOM ! Freedom (.l.) ")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.publisher_cmd.publish(move_cmd)

            #Update this attribute
            self.isOut=True

            #Puplish finish
            finish=Bool()
            finish.data=True
            self.publisher_finish.publish(finish)
        
        #message topic received =False
        elif not msg:
            self.get_logger().info(f"Received Topic /tp3/start with False =>> Doing Nothing !")
        else :
            self.get_logger().info(f"The Robot is already outside  room =>> Doing Nothing !")
            



    def create_marker(self,mid,point,red,green,bleu):
        marker = Marker()

        # define the frame of the marker (needed to be displayed)
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        # Set the namespace and id for this marker.  This serves to create a unique ID
        # Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes"
        marker.id = mid

        # Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = Marker.CUBE

        # Set the marker action.  Options are ADD and DELETE
        marker.action = Marker.ADD

        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = .5
        marker.scale.y = .5
        marker.scale.z = .5

        # Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = red
        marker.color.g = green
        marker.color.b = bleu
        marker.color.a = 1.0

        # marker.lifetime = 0

        # Publish the marker
        self.pub_marker.publish(marker)

        
    def find_orientation(self,angle):
        if -0.785<=angle and angle <=0.785:#[-45,45]
            return "N"
        elif -2.356 <= angle and angle <=-0.785:#[-135,-45]
            return "E"
        elif 0.785 <=angle and angle <=2.35:#[45,135]
            return "W"
        else :
            return "S"

def main(args=None):
    rclpy.init(args=args)
    my_node = EscapeRoom()
    #my_node.get_logger().info("Beginning escaping room ...")
    # declaration of the multithreaded executor
    executor = MultiThreadedExecutor()
    # adding the node to the executor
    executor.add_node(my_node)
    # spinning the executor
    try:
        executor.spin()
    except KeyboardInterrupt as ki: #Eviter l'erreur KeyboardInterrupt
        pass
    
    
    #rclpy.shutdown()


if __name__ == '__main__':
    main()


#ros2 topic pub --once /tp3/start std_msgs/msg/Bool "{data : true }"


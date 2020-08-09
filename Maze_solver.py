#!/usr/bin/env python
 
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
import time
 
class GetMeOut():
 
    def __init__(self):
        rospy.init_node('Turtlebot', anonymous=True)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.Laser_callback)
        self.Gyro_sub = rospy.Subscriber('/odom', Odometry, self.Odom_callback)
        self.twist = Twist()
        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
        self.Desired_Angle=0
        self.first_turn = True
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.GetOut)
############### The Main Function of the Demo #################
    
    def planToEscape(self):
        time.sleep(0.5) 
        # this delay because when the program starts
        # there is no  data readed from the laserScan sensor yet so we avoid such an error "syntax Error : List out of index"
        Front = self.laser_msg.ranges[360]
        while True:
            while (Front> 0.9):
                self.twist.linear.x = 0.8
                self.twist_Publisher()
                Front = self.laser_msg.ranges[360]
                Left=self.laser_msg.ranges[719]
                Right=self.laser_msg.ranges[0]
                print("Distance front of me :"+ str(Front)+" Right :" +str(Right)+" Left:"+str(Left))
                if Left>100 and Right>100: 
                 exit(1)
                 self.GetOut()
            self.stop()
            self.turn()
            Front=self.laser_msg.ranges[360]
 ########################### Sensors callback funcs###################################
    def Laser_callback(self, command):
    # This is the callback func of the sensor "Laser "
    # which updates the global vari laser_command : with sensor new data
        self.laser_msg = command
    

    
    def Odom_callback(self, command):
        # This is the callback func of the sensor "Odometery "
        # which updates the global vari Gyro_command : with sensor new data
        self.odom_msg = command
############################### Correcting path funcs ######################################################

    def rightOrLeft(self): # To determine which direction should robot turns to
      if (max(self.laser_msg.ranges[:360]) > max(self.laser_msg.ranges[360:])):
        direction = 'R'
      else:
        direction = 'L'
      return direction
    
    def turn(self):  

        direction = self.rightOrLeft()
        # For the code to be valid for any starting position 
        if (self.first_turn):
          self.Desired_Angle = euler_from_quaternion([self.odom_msg.pose.pose.orientation.x,
            self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w])[2]    
          self.first_turn=False
        if (-0.2< self.Desired_Angle <0.2):
            self.Desired_Angle=0
 
        if (direction =='R'): 
            angle = -1 # allow the while loop to start
            # if the robot to turn right we subtract 90 degrees from it's current angle
            # turning right from (0 --> -180)
            # if the robot reached -180 that means we need to add 90 to keep in range (-180:180)
            if (self.Desired_Angle== -math.pi):

                # keeping the angle in the range of (-180 to 180).
                self.Desired_Angle -= math.pi/2 - math.pi*2
            else:
                # if the current angle is not reached -180 yet, keep subtracting -90 from robot angle
                self.Desired_Angle -= math.pi/2 

 
        else:
            angle = 1  # allow the while loop to start
            #  by adding 90 degrees to robot angle we are turning left 
            # which is the angle the robot is at right now.
            if (self.Desired_Angle == math.pi):
                # keeping the angle in the range (-180 to 180).
                self.DesiredAngle += math.pi/2 - math.pi*2
            else:
                # the right direction is from 0 to 180.
                self.Desired_Angle += math.pi/2
 
        while not (-0.01<= angle <= 0.01):  # that's mean the robot current angle reached desired angle
            orientation = self.odom_msg.pose.pose.orientation
            yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
            angle = self.Desired_Angle- yaw
            if direction =='R':
                # the angle is  negative when wanting to turn right.
                angle = - abs(angle)
                print("correcting path ...")
                print("turning right")
            else:
                # the angle is positive when wanting to turn left.
                angle = abs(angle)
                print("correcting path ...")
                print("turning left")
            self.twist.angular.z = angle
            self.twist_Publisher()
            self.rate.sleep()
        self.stop()
#########################################################################
    def twist_Publisher(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important to make sure that the publisher is available first.
        """
        while not self.ctrl_c:
            connections = self.twist_publisher.get_num_connections()
            # if connections is equal 0, then the publisher is not ready.
            # if connections is equal 1, then the publisher is ready.
            if connections > 0:
                self.twist_publisher.publish(self.twist)
                break
            else:
                self.rate.sleep()

    def GetOut(self):
    # this function responsible for closing the program 
    #  as soon as the robot get out of the maze
        self.stop()
        print("oooh finally ! I am Outtttttttt ")
        self.ctrl_c = True
    
    def stop(self): #func to stop robot movement
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        print("I will stop now to turn ..")
        self.twist_Publisher()
 
if __name__ == '__main__':

    robot = GetMeOut()
    robot.planToEscape()


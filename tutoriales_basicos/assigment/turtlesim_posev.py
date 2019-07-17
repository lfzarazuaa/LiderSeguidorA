#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x=0
y=0
z=0
yaw=0

def poseCallback(pose_message):
    global x
    global y, z, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta


def move(speed, distance):
            #declare a Twist message to send velocity commands
            velocity_message = Twist()
            #get current location from the global variable before entering the loop 
            x0=x
            y0=y
            #z0=z;
            #yaw0=yaw;

            #task 1. assign the x coordinate of linear velocity to the speed. 
            velocity_message.linear.x=speed
             
            distance_moved = 0.0
            loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    

            #task 2. create a publisher for the velocity message on the appropriate topic.  
            velocity_publisher =  rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
            while True :
                 rospy.loginfo("Turtlesim moves forwards")
                 #task 3. publish the velocity message 
                 velocity_publisher.publish(velocity_message)
                 loop_rate.sleep()
                 #rospy.Duration(1.0)
                 #measure the distance moved
                 distance_moved = distance_moved+abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
                 print  distance_moved               
                 if  not (distance_moved<distance):
                     rospy.loginfo("reached")
                     break
            #task 4. publish a velocity message zero to make the robot stop after the distance is reached
            velocity_message.linear.x=0
            velocity_publisher.publish(velocity_message)

def Turtle():
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        #task 5. declare velocity publisher
        vel_publisher =  rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)
        print 'move: '
        move (1.0, 5.0)
        time.sleep(2)
        print 'start reset: '
        rospy.wait_for_service('reset')
        reset_turtle = rospy.ServiceProxy('reset', Empty)
        reset_turtle()
        print 'end reset: '
        rospy.spin()
        
if __name__ == '__main__':
    try:
        Turtle()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
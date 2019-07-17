#!/usr/bin/env python
# -*- coding: utf-8 -*-
#what is the topic that makes the robot move
#suscriber for the topic that will show the location of the robot
#publisher to the topic that will make the robot move
#what is the topic of the position 
#develop a method called move(distance)
#which makes the robot moves a certain distance then stops. 
#develop a method called rotate(distance) 
#which makes the robot rotate a certain angle then stops.
import rospy
import math
#from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
def move():
    print "Hola"

def rotate():
    print "Hola"

def ObtainPose(data):
    pose = data
    PoseStr="X Position:%f Y Position:%f" % (pose.x,pose.y)
    VelStr="Linear:%f Angular:%f" % (pose.linear_velocity,pose.angular_velocity)
    rospy.loginfo(PoseStr)
    rospy.loginfo(VelStr)
    
def change_vel(vlinear,vangular):
        twist = Twist()
        twist.linear.x = vlinear
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = vangular
        return twist

def movement():
    pub_twist = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    lis_pose = rospy.Subscriber('/turtle1/pose', Pose, ObtainPose)
    rospy.init_node('movement', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    i=1
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        i=i+1
        if i>4:
           i=1
        pub_twist.publish(change_vel(i,3));
        rate.sleep()

if __name__ == '__main__':
    try:
        movement()
    except rospy.ROSInterruptException:
        pass
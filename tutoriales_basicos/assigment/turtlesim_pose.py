#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
#task 1. import the Pose type from the module turtlesim


def ObtainPose(pose_message):
    pose = pose_message
    #pose = Pose()
    #task 4. display the x, y, and theta received from the message
    print ("pose callback")
    print ('x = %f' % pose.x) 
    print ('y = %f' % pose.y) 
    print ('yaw = %f \n' % pose.theta) 

if __name__ == '__main__':
    try:
     #task 2. subscribe to the topic of the pose of the Turtlesim
     lis_pose = rospy.Subscriber('/turtle1/pose', Pose, ObtainPose)
     rospy.init_node('turtlesim_motion_pose', anonymous=True)        
     #task 3. spin
     rate = rospy.Rate(2) # 2hz
     while not rospy.is_shutdown():
        rate.sleep() 
           
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
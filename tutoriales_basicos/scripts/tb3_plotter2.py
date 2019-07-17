#!/usr/bin/env python
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from turtlesim.msg import Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Plotter:
    
    def __init__(self):
      self.poseflag=False
      self.pose_subscriber=rospy.Subscriber("/tb3_1/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)
      self.turtlebot3_pose=Pose()
      self.Positions_X=[]
      self.Positions_Y=[]
      #self.fig = plt.figure(figsize=(7,7), facecolor='w')
      #self.fig.canvas.set_window_title('Trayectorias generadas')
      plt.ion()

    def poseCallback(self,data):
        self.turtlebot3_pose.x=data.pose.pose.position.x
        self.turtlebot3_pose.y=data.pose.pose.position.y
        self.Positions_X.append(self.turtlebot3_pose.x)
        self.Positions_Y.append(self.turtlebot3_pose.y)
        plt.plot(self.Positions_X,self.Positions_Y, '*')
        plt.axis([-1.25,1.25,-1.25,1.25])
        plt.draw()
        plt.pause(0.00000000001)
        self.poseflag=True               
    

def main():
        rospy.init_node('Plotter', anonymous=True)
        P=Plotter() # constructor creates publishers / subscribers
        print 'PLotter inicializado'
        rospy.spin()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

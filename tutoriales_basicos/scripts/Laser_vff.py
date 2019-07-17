#!/usr/bin/env python

import rospy
import numpy as np
from math import cos,sin,radians
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

map_size_x=90.0 #cm
map_size_y=90.0 #cm
resolution = 1.0 #cm

class ForceField():
    def __init__(self):
        self.poseflag=False
        self.turtlebot3_pose=Pose()
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.PoseCallback, queue_size=1)  #Posicion lider
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.LaserCallback, queue_size=1)  #Posicion lider
        self.matrix = np.zeros((int(map_size_x/resolution),int(map_size_y/resolution)),dtype='f')
        self.lim_x,self.lim_y = (int(map_size_x/resolution),int(map_size_y/resolution))
        self.count=1
        self.rate = rospy.Rate(10) # 10hz
        self.loop()

    def PoseCallback(self,data):
        self.turtlebot3_pose.x=data.pose.pose.position.x
        self.turtlebot3_pose.y=data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.turtlebot3_pose.theta=self.convert2pi(yaw)*180/np.pi
        #print 'yaw=',self.turtlebot3_pose.theta*180/np.pi
        self.poseflag=True
    
    def convert2pi(self,theta):
        if theta<0:
            return 2*np.pi+theta
        else:
            return theta

    def LaserCallback(self,data):
        if self.poseflag==True:
            self.middle_x,self.middle_y=(int(map_size_x/(2*resolution)),int(map_size_y/(2*resolution)))
            print 'middle_x=',self.middle_x,'middle_y=',self.middle_y
            #print 'angle=',int(self.turtlebot3_pose.theta)
            for i in range(0,360,1):
                #print 'angle=',i,'index=',self.obtainIndexAngle(i)
                index=self.obtainIndexAngle(i)
                dist=data.ranges[index]*100
                #print 'angle=',i,'dist=',dist
                if dist<=int(map_size_x)/2 and dist<=int(map_size_y)/2: #Guardar dato en la matriz
                    xi=dist*np.cos(radians(i))
                    yi=dist*np.sin(radians(i))
                    index_x=self.middle_x+int(xi/resolution)
                    index_y=self.middle_y+int(yi/resolution)
                    #an=self.convert2pi(np.arctan2(index_y-middle_y,index_x-middle_x))*180/np.pi
                    #print 'angle=',i,'dist=',dist,'index x=',index_x,'index y=',index_y #,'angle=',an
                    self.matrix[index_x,index_y]=self.matrix[index_x,index_y]+1
            if self.count>=3:
                Fx,Fy,Fa=self.calculateForce()
                print 'Fx=',Fx,'Fy=',Fy,'Fa=',Fa*180/np.pi
                self.matrix = np.zeros((int(map_size_x/resolution),int(map_size_y/resolution)),dtype='f')
                self.count=0
            self.count=self.count+1

    def obtainIndexAngle(self,angle):
        index=(360+angle-int(self.turtlebot3_pose.theta))%(360)
        #index=(360+angle+int(self.turtlebot3_pose.theta))%(360)
        return index

    def calculateForce(self):
        sumx=0.0
        sumy=0.0
        for xi in range(0, self.lim_x):
            for yi in range(0, self.lim_y):
                if self.matrix[xi,yi]>0:
                    c=self.matrix[xi,yi]
                    an=self.convert2pi(np.arctan2(yi-self.middle_y,xi-self.middle_x))
                    sumx=sumx-c*np.cos(an)
                    sumy=sumy-c*np.sin(an)
        Magnitude=np.sqrt(sumx**2+sumy**2)
        print 'sumx=',sumx,'sumy=',sumy,'Mag=',Magnitude
        Angle=self.convert2pi(np.arctan2(sumy,sumx))
        Fx=sumx/Magnitude
        Fy=sumy/Magnitude
        Force= (Fx,Fy,Angle)
        return Force

    def loop(self):
        while not rospy.is_shutdown():
            pass

def main():
        rospy.init_node('Vector_Force_Field', anonymous=True)
        FF=ForceField() # constructor creates publishers / subscribers
        print 'Nodo inicializado'

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
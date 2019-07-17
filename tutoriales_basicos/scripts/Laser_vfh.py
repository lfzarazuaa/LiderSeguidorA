#!/usr/bin/env python
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose
from tutoriales_basicos.msg import Histogram

from tf.transformations import euler_from_quaternion, quaternion_from_euler

min_radius = 45.0 #cm
m=2
n=1
s=30
class LaserVFH:
    
    def __init__(self):
        self.poseflag=False
        self.LIDAR_ERR=0.004
        self.turtlebot3_pose=Pose()
        self.pose_sub = rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped, self.PoseCallback, queue_size=1)  #Posicion lider
        self.laser_sub = rospy.Subscriber("/tb3_0/scan", LaserScan, self.LaserCallback2, queue_size=1)  #Posicion lider
        self.histogram_publisher=rospy.Publisher('/tb3_0/histogram', Histogram, queue_size=1)
        self.histogram=Histogram()
        self.a = 5
        self.b = self.a/(min_radius**2)
        self.dmax=2*min_radius
        plt.ion()
        self.plot=False
        #plt.title("Polar Histogram")
        #plt.xlabel("Angle")
        #plt.ylabel("Distance")
        

    def PoseCallback(self,data):
        self.turtlebot3_pose.x=data.pose.pose.position.x
        self.turtlebot3_pose.y=data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.turtlebot3_pose.theta=self.convert2pi(yaw)*180/np.pi
        self.poseflag=True         
    
    def convert2pi(self,theta):
        if theta<0:
            return 2*np.pi+theta
        else:
            return theta
    
    def LaserCallback3(self,data):
        if self.poseflag==True:
            an=np.zeros(360,dtype='f')
            d=np.zeros(360,dtype='f')
            H=np.zeros(360,dtype='f')
            Hk=np.zeros(360,dtype='f')
            h=np.zeros(360,dtype='f')
            for i in range(0,360,1):
                an[i]=i
                dist=data.ranges[self.obtainIndexAngle(i)]*100
                if dist<=min_radius:
                    d[i]=dist
                else:
                    d[i]=self.dmax
                H[i]=2*(self.a-self.b*d[i]**2)
                if H[i]>0:
                    Hk[i]=1
                else:
                    Hk[i]=0
            h[0]=Hk[0]-Hk[359]
            for i in range(1,360,1):
                h[i]=Hk[i]-Hk[i-1]
            print 'min=',min(H)
            print 'max=',max(H)
            # plt.subplot(m,n,1)
            # plt.hold(False)
            # plt.plot(an,H)
            # plt.axis([0,360,-50,30])
            # plt.draw()
            # plt.pause(0.00000000001)
            # plt.subplot(m,n,2)
            # plt.hold(False)
            # plt.plot(an,h)
            # plt.axis([0,360,-2,2])
            # plt.draw()
            # plt.pause(0.00000000001)
               
    def LaserCallback2(self,data):
        if self.poseflag==True:
            an=np.zeros(360,dtype='f')
            d=np.zeros(360,dtype='f')
            H=np.zeros(360,dtype='f')
            Hk=np.zeros(360,dtype='f')
            h=np.zeros(360,dtype='f')
            for i in range(0,360,1):
                an[i]=i
                dist=data.ranges[self.obtainIndexAngle(i)]*100
                if dist<=min_radius and dist>=self.LIDAR_ERR:
                    H[i]=1
                else:
                    H[i]=0
            h[0]=H[0]-H[359]
            for i in range(1,360,1):
                h[i]=H[i]-H[i-1]
            self.an_begin = []
            self.an_end = []
            for i in range(1,360,1):
                if h[i]>0.5:
                    self.an_begin.append(i)
                elif h[i]<-0.5:
                    self.an_end.append(i)
            if self.plot==True:
                plt.subplot(m,n,1)
                plt.hold(False)
                plt.plot(an,H)
                plt.axis([0,360,-2,2])
                plt.title("Histograma Polar")
                plt.draw()
                plt.pause(0.00000000001)
            l=len(self.an_begin)
            if l>0:
                for i in range(0,l,1):
                    ind_back=self.an_begin[i]
                    for k in range (0,-s,-1):
                        ind=(ind_back+k)%360
                        H[ind]=1
            l=len(self.an_end)
            if l>0:
                for i in range(0,l,1):
                    ind_forw=self.an_end[i]
                    for k in range (0,s,1):
                        ind=(ind_forw+k)%360
                        H[ind]=1
            self.histogram=H
            self.histogram_publisher.publish(self.histogram)
            #print 'valores=',h
            #if self.an_end[0]<self.an_begin[0]
            #   self.an_end=shift_left(self.an_end,1)
            # print 'inicios=',self.an_begin
            # print 'finales=',self.an_end
            # #ang=(s+360+self.an_begin)%360
            if self.plot==True:
                plt.subplot(m,n,2)
                plt.hold(False)
                plt.plot(an,H)
                plt.axis([0,360,-1.5,1.5])
                plt.title("Histograma Polar Modificado")
                plt.draw()
                plt.pause(0.00000000001)            
                
    def obtainIndexAngle(self,angle):
        index=(360+angle-int(self.turtlebot3_pose.theta))%(360)
        #index=(360+angle+int(self.turtlebot3_pose.theta))%(360)
        return index

def main():
        rospy.init_node('Laser_VFH', anonymous=True)
        L=LaserVFH() # constructor creates publishers / subscribers
        print 'VFH inicializado'
        rospy.spin()
        
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
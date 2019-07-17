#!/usr/bin/env python
import sys
import rospy
import os
import numpy as np

import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose
from tutoriales_basicos.msg import Histogram
from std_msgs.msg import Float64,Int32,Bool

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ForceController:
    
    def __init__(self):
      self.poseflag=False
      self.turtlebot3_pose=Pose()
      self.goal_pose=Pose()
      self.histogram=[]
      self.vel_msg=Twist()
      #set in zero linear velocities
      self.vel_msg.linear.x=0
      self.vel_msg.linear.y=0
      self.vel_msg.linear.z=0
      #set in zero angular velocities
      self.vel_msg.angular.x=0
      self.vel_msg.angular.y=0
      self.vel_msg.angular.z=0
      #set map propierties
      self.map_size_x=250 #cm
      self.map_size_y=250 #cm
      self.resolution = 1 # cm
      self.lineal=0.08#.1 #velocidad lineal
      self.lim_angular=1.2 #limite velocidad angular.
      self.obs_min_dist=0.13+0.105*2.0
      self.fdvh=0.35
      self.fd=1.0#1.3
      self.dist=0.030
      self.dist_min=0.5
      self.lane=0.0
      self.turn_on=False
      self.al=True
      ruta=os.path.dirname(os.path.abspath(__file__))
      self.matrixA = np.load(ruta+'/TrayA1.npy')
      self.matrixB = np.load(ruta+'/TrayB1.npy')
      self.matrixC = np.load(ruta+'/TrayC1.npy')
      #Creates publishers and subscribers
      self.pose_subscriber=rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)
      self.histogram_subscriber=rospy.Subscriber("/tb3_0/histogram", Histogram,self.histogramCallback,queue_size=1)
      self.lane_subscriber=rospy.Subscriber("/lane",Int32,self.laneCallback,queue_size=1)
      self.turn_on_subscriber=rospy.Subscriber("/turn_on",Bool,self.turn_onCallback,queue_size=1)
      self.dist_min_subscriber=rospy.Subscriber("/tb3_0/dist_min", Float64,self.dist_minCallback,queue_size=1)
      #self.min_dist_subscriber=rospy.Subscriber("/tb3_0/dist_min", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)
      self.velocity_publisher=rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
      self.rate = rospy.Rate(10) # 10hz
      while self.poseflag==False:
          self.setStop()
      while self.turn_on==False:
          self.setStop()
      
    def poseCallback(self,data):
        self.turtlebot3_pose.x=data.pose.pose.position.x
        self.turtlebot3_pose.y=data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.turtlebot3_pose.theta=self.convert2pi(yaw)
        #print 'yaw=',self.turtlebot3_pose.theta*180/np.pi
        self.poseflag=True  

    def histogramCallback(self,data):
        self.histogram=list(data.Histogram)
        #print self.h[0],self.h[0]==0

    def laneCallback(self,data):
        self.lane=data.data

    def turn_onCallback(self,data):
        self.turn_on=data.data
        if self.turn_on==False:
            self.setStop()
    
    def dist_minCallback(self,data):
        self.dist_min=data.data

    def convert2pi(self,theta):
        if theta<0:
            theta=2*np.pi+theta
        else:
            theta=theta
        return theta

    def setStop(self):
        self.vel_msg.linear.x=0
        self.vel_msg.angular.z=0
        self.velocity_publisher.publish(self.vel_msg)
    
    def getDistance(self,x1,x2,y1,y2):
    	return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def getFlag(self):
        return self.poseflag

    def getDistance2(self,d1,d2):
        return np.sqrt(np.power(d2.x-d1.x,2)+np.power(d2.y-d1.y,2))

    def getAngle(self,d1,d2):
        return self.convert2pi(np.arctan2(d2.y-d1.y,d2.x-d1.x))

    def moveGoal(self,goal_pose,distance_tolerance):
        self.rate = rospy.Rate(15)
        #****** Proportional Controller ******#
		#    linear velocity in the x-axis
        kl=0.3#0.22
        ka=1.5#1.5
        while True:
              dist=self.getDistance2(self.turtlebot3_pose,goal_pose)
              if not(dist>distance_tolerance):
                   break
              xvel=kl*dist
              if(xvel>0.22):
                  xvel=0.22
              self.vel_msg.linear.x=xvel
              self.vel_msg.linear.y=0
              self.vel_msg.linear.z=0
              #set a ka proportional angular velocity in the z-axis
              self.vel_msg.angular.x=0
              self.vel_msg.angular.y=0
              self.vel_msg.angular.z =ka*(self.getAngle(self.turtlebot3_pose,goal_pose)-self.turtlebot3_pose.theta)
              self.velocity_publisher.publish(self.vel_msg)
              self.rate.sleep()
        #make zero the linear and angular velocity
        #self.vel_msg.linear.x=0
        #self.vel_msg.angular.z=0
        #self.velocity_publisher.publish(self.vel_msg)
        print 'x=',self.turtlebot3_pose.x,'y=',self.turtlebot3_pose.y

    def moveGoal2(self,goal_pose,distance_tolerance):
        self.rate = rospy.Rate(15)
        #****** Proportional Controller ******#
		#    linear velocity in the x-axis
        kl=0.3#0.22
        ka=1.0#1.5
        while True:
              dist=self.getDistance2(self.turtlebot3_pose,goal_pose)
              if not(dist>distance_tolerance):
                   break
              lvel=self.lineal#kl*dist
              a=(self.getAngle(self.turtlebot3_pose,goal_pose)-self.turtlebot3_pose.theta)
              avel=(a+np.pi)%(2*np.pi)-np.pi
              if abs(avel)>45*np.pi/180:
                  lvel=0.0
              avel=ka*avel
              if avel>self.lim_angular:
                  avel=self.lim_angular
              elif avel<-self.lim_angular:
                  avel=-self.lim_angular
              self.vel_msg.linear.x=lvel
              self.vel_msg.linear.y=0
              self.vel_msg.linear.z=0
              #set a ka proportional angular velocity in the z-axis
              self.vel_msg.angular.x=0
              self.vel_msg.angular.y=0
              self.vel_msg.angular.z =avel
              if self.turn_on==False:
                self.setStop()
              else:
                  self.velocity_publisher.publish(self.vel_msg)
              self.rate.sleep()
        print 'x=',self.turtlebot3_pose.x,'y=',self.turtlebot3_pose.y

    def move2angle(self,dist_y,dist_x):
        self.rate = rospy.Rate(10)
        ka=1.0
        a=self.convert2pi(np.arctan2(dist_y,dist_x))-self.turtlebot3_pose.theta
        avel=(a+np.pi)%(2*np.pi)-np.pi
        if abs(avel)>45*np.pi/180:
            lvel=0.0
        else:
            lvel=self.lineal
        avel=ka*avel
        if avel>self.lim_angular:
            avel=self.lim_angular
        elif avel<-self.lim_angular:
            avel=-self.lim_angular
        #set a lineal velocity
        self.vel_msg.linear.x=lvel
        #set a ka proportional angular velocity in the z-axis
        self.vel_msg.angular.z =avel
        self.velocity_publisher.publish(self.vel_msg)
        self.rate.sleep()

    def moveBangle(self,an):
        self.rate = rospy.Rate(3)
        ka=1.0
        actual=self.turtlebot3_pose.theta
        a=((an*np.pi/180)-actual)
        avel=(a+np.pi)%(2*np.pi)-np.pi
        print 'obj=',an,'actual=',actual*180/np.pi,'error=',a*180/np.pi,'mov=',avel*180/np.pi
        if abs(avel)>45*np.pi/180:
            lvel=0
        else:
            lvel=self.lineal
        avel=ka*avel
        if avel>self.lim_angular:
            avel=self.lim_angular
        elif avel<-self.lim_angular:
            avel=-self.lim_angular
        #set a lineal velocity
        self.vel_msg.linear.x=lvel
        #set a ka proportional angular velocity in the z-axis
        self.vel_msg.angular.z =avel
        self.velocity_publisher.publish(self.vel_msg)
        self.rate.sleep()

    def follow(self):
        if self.turn_on==False:
            self.setStop();
            return
        x1=self.turtlebot3_pose.x
        y1=self.turtlebot3_pose.y
        x_index=np.int((x1+1.25)*(100/self.resolution))#obtener el indice en x
        y_index=np.int((y1+1.25)*(100/self.resolution))#obtener el indice en y
        #Definir limites del indice en x
        if (x_index<0):
            x_index = 0
        elif (x_index>((self.map_size_x/self.resolution)-1)):
            x_index=(self.map_size_x/self.resolution)-1
        #Definir limites del indice en y
        if (y_index<0):
            y_index = 0
        elif (y_index>((self.map_size_y/self.resolution)-1)):
            y_index=(self.map_size_y/self.resolution)-1

        if (self.lane==1):
          dist_x, dist_y = self.matrixA[x_index,y_index,:]#Obtener la distancia a la que se quiere llegar.
        elif(self.lane==2):
          dist_x, dist_y = self.matrixB[x_index,y_index,:]#Obtener la distancia a la que se quiere llegar.
        elif(self.lane==3):
          dist_x, dist_y = self.matrixC[x_index,y_index,:]#Obtener la distancia a la que se quiere llegar.
        else:
            self.setStop();
            return
        
        obs_dist = self.dist_min#rospy.wait_for_message("/tb3_0/dist_min", Float64) #Subscriptor de una sola vez.
        min_dist=0.025
        if obs_dist>self.obs_min_dist:
            if self.turn_on==False:
                self.setStop()
            else:
                self.move2angle(dist_y,dist_x)
            #x2=x1+dist_x/self.fd
            #y2=y1+dist_y/self.fd
            #print 'x1=',x1,'x2=',x2,'y1=',y1,'y2=',y2
            #self.goal_pose.x=x2
            #self.goal_pose.y=y2
            #self.moveGoal2(self.goal_pose,min_dist)#0.025
        else:
            print 'Obteniendo polar hist dist=',obs_dist
            self.obtainGoal(dist_x,dist_y,x1,y1,min_dist)
            

    def obtainGoal(self,dx,dy,x1,y1,min_dist):
        ang=self.convert2pi(np.arctan2(dy,dx))
        an=int(ang*180/np.pi)
        if self.histogram[an]==1:
            #Modifica ruta
            #dist=0.025*4
            h=1
            ind=an
            while h==1:
                ind=(ind+1)%360
                h=self.histogram[ind]
            print 'Modificar angulo ',an,' por ',ind
            if self.turn_on==False:
                self.setStop()
            else:
                if self.al==True:
                    if self.dist_min<0.13+0.105*1.25:
                        an=(ind+15)%360
                    else:
                        an=ind
                    self.moveBangle(an)
                else:
                    an=ind*np.pi/180
                    x2=x1+np.cos(an)*self.fdvh
                    y2=y1+np.sin(an)*self.fdvh
                    self.goal_pose.x=x2
                    self.goal_pose.y=y2
                    self.moveGoal2(self.goal_pose,min_dist)#0.025
                
                
        else:
            if self.turn_on==False:
                self.setStop()
            else:
                self.move2angle(dy,dx)
            

        
def main():
        rospy.init_node('ForceController', anonymous=True)
        FC=ForceController() # constructor creates publishers / subscribers
        print 'Nodo Lider inicializado'
        while (not rospy.is_shutdown()):
            FC.follow()
        FC.setStop()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

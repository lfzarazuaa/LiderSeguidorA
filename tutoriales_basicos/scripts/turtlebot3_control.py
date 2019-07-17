#!/usr/bin/env python
import sys
import rospy
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ForceController:
    
    def __init__(self):
      self.poseflag=False
      self.turtlebot3_pose=Pose()
      self.goal_pose=Pose()
      self.goal=PoseStamped()
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
      self.resolution = 1 #cm
      self.lineal=0.07#.1 #velocidad lineal
      self.lim_angular=0.22 #limite velocidad angular.
      self.lane=1
      if (self.lane==1):
          self.matrix = np.load('/home/zarazua_rt/catkin_ws/src/tutoriales_basicos/scripts/TrayA1.npy')
      elif(self.lane==2):
          self.matrix = np.load('matrixDynamic_lane2.npy')
      else:
          self.matrix = np.load('matrixDynamic_lane2.npy')
      self.velocity_publisher=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
      self.pose_subscriber=rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)
      self.rate = rospy.Rate(10) # 10hz
      
      
    
    def poseCallback(self,data):
        self.turtlebot3_pose.x=data.pose.pose.position.x+1.25
        self.turtlebot3_pose.y=data.pose.pose.position.y+1.25
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.turtlebot3_pose.theta=yaw
        print 'Posicion recibida'
        self.poseflag=True

    def getAngle(self,d1,d2):
        return np.arctan2(d2.y-d1.y,d2.x-d1.x)

    def move(self):
        x1=self.turtlebot3_pose.x
        y1=self.turtlebot3_pose.y
        x_index=np.int(x1*(100/self.resolution))#obtener el indice en x
        y_index=np.int(y1*(100/self.resolution))#obtener el indice en y
        #print 'xind=',x_index,'yind=',y_index
        yaw=self.turtlebot3_pose.theta #Guardar el angulo yaw
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

        x3, y3 = self.matrix[x_index,y_index,:]#Obtener la distancia a la que se quiere llegar.
        x2=x1+x3
        y2=y1+y3
        
        Ka=4.0
        distance_tolerance=0.05
        while True:
            x1=self.turtlebot3_pose.x
            y1=self.turtlebot3_pose.y
            yaw=self.turtlebot3_pose.theta #Guardar el angulo yaw
            dist=self.getDistance(x1,x2,y1,y2)
            goal_angle=np.arctan2(y2-y1,x2-x1)
            #Orientar la posicion del carro a la del sistema de referencia para trabajar en un plano absoluto.
            #f_x=np.cos(yaw)*x3 + np.sin(yaw)*y3#Rotacion en X.
            #f_y=-np.sin(yaw)*x3 + np.cos(yaw)*y3#Rotacion en Y
            #print 'fx=',f_x,'fy=',f_y
            
            print 'dist=',dist,'xi=',x1-1.25,'yi=',y1-1.25,'xf=',x2-1.25,'yf=',y2-1.25,'goal=',goal_angle,'yaw=',yaw    
            angular=Ka*(goal_angle-yaw)#Ka*np.arctan(f_y/(2.5*f_x))
            #print 'goal=',goal_angle,'yaw=',yaw,'angular=',angular
            if angular>self.lim_angular:
                angular=self.lim_angular
            elif angular<-self.lim_angular:
                angular=-self.lim_angular
            #set and publish velocities
            self.vel_msg.linear.x=self.lineal
            self.vel_msg.angular.z=angular
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
            if not(dist>distance_tolerance):
                   break
        print 'Distancia alcanzada'                   
    
    def setStop(self):
        self.vel_msg.linear.x=0
        self.vel_msg.angular.z=0
        self.velocity_publisher.publish(self.vel_msg)
    
    def getDistance(self,x1,x2,y1,y2):
    	return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def getFlag(self):
        return self.poseflag

def main():
        rospy.init_node('ForceController', anonymous=True)
        FC=ForceController() # constructor creates publishers / subscribers
        print 'Nodo inicializado'
        while FC.getFlag()==False:
            pass
        print FC.getFlag()
        t0=rospy.get_time()
        while FC.getFlag()==True:
            FC.move()
            t1=rospy.get_time()
            if not(t1-t0<30):
                break
        FC.setStop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import numpy as np
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16


class ForceController:
    def __init__(self):
        self.flag=False
        self.map_size_x=250 #cm
        self.map_size_y=250 #cm
        self.resolution = 10 # cm
        self.lineal=0.5 #velocidad lineal
        self.lim_angular=0.22 #limite velocidad angular.
        self.lane=1
        if (self.lane==1):
        	self.matrix = np.load('/home/zarazua_rt/catkin_ws/src/tutoriales_basicos/scripts/matrixDynamic_lane1.npy')
        elif(self.lane==2):
        	self.matrix = np.load('matrixDynamic_lane2.npy')
        else:
            self.matrix = np.load('matrixDynamic_lane2.npy')
        self.flag=True
        self.pose_subscriber=rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)
        self.velocity_publisher=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
    # def lane_callback(self, data):
    # 	if (self.lane==1):
    # 		self.lane=2
    # 		self.matrix = np.load('trayectorias/matrixDynamic_lane2.npy')
    # 	else:
    # 		self.lane=1
    # 		self.matrix = np.load('trayectorias/matrixDynamic_lane1.npy')

    def poseCallback(self, data):
        # x = data.pose.pose.position.x + 1.25 #Leer posicion amcl de x
        # y = data.pose.pose.position.y + 1.25 #Leer posicion amcl de y
        # orientation_q = data.pose.pose.orientation #obtner la estructura de orientacion
        # #Crear lista de orientacion
        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # #Obtener los angulos x,y,z de rotacion 
        # (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        # x_index=np.int(x*self.resolution)#obtener el indice en x
        # y_index=np.int(y*self.resolution)#obtener el indice en y
        
        # #Definir limites del indice en x
        # if (x_index<0):
        #     x_index = 0
        # elif (x_index>((self.map_size_x/self.resolution)-1)):
        #     x_index=(self.map_size_x/self.resolution)-1

        # #Definir limites del indice en y
        # if (y_index<0):
        #     y_index = 0
        # elif (y_index>((self.map_size_y/self.resolution)-1)):
        #     y_index=(self.map_size_y/self.resolution)-1

        # x3, y3 = self.matrix[x_index,y_index,:]#Obtener la distancia a la que se quiere llegar.
        # print(x3, y3)#Imprimir las distancias.
        # print(yaw)#Imprimir el angulo en el que se esta.
        # #Orientar la posicion del carro a la del sistema de referencia para trabajar en un plano absoluto.
        # f_x=np.cos(yaw)*x3 + np.sin(yaw)*y3#Rotacion en X.
        # print(f_x)#Posicion en x el plano absoluto.
        
        # f_y=-np.sin(yaw)*x3 + np.cos(yaw)*y3#Rotacion en Y
        # Ka=4.0
        # angular=Ka*np.arctan(f_y/(f_x))#Ka*np.arctan(f_y/(2.5*f_x))
        
        # if angular>self.lim_angular:
        #     angular=self.lim_angular
        # elif angular<-self.lim_angular:
        #     angular=-self.lim_angular
        #Asignar valores y publicar la velocidad  
        angular=0.5  
        self.vel_msg=Twist()
        self.vel_msg.linear.x=self.lineal
        self.vel_msg.linear.y=0
        self.vel_msg.linear.z=0
        #set a random angular velocity in the z-axis
        self.vel_msg.angular.x=0
        self.vel_msg.angular.y=0
        self.vel_msg.angular.z=angular
        print 'Antes de publicar'
        self.velocity_publisher.publish(self.vel_msg)
        print 'publicado'
        return 0


def main():
    rospy.init_node('ForceController')
    ForceController()  # constructor creates publishers / subscribers
    rospy.spin()

if __name__ == '__main__':
    main()
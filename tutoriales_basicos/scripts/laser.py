#!/usr/bin/env python

import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from tutoriales_basicos.msg import Histogram
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16
from std_msgs.msg import Float32


BURGER_MAX_LIN_VEL = 0.22*.7
BURGER_MAX_ANG_VEL = 2.84

#import matplotlib.pyplot as plt
def min(c1,c2):
    s = 4
    return(np.amin([(c1 - c2),(c1 - c2 - s),(c1 - c2 + s)]))


class LaserSub:
    def __init__(self):
        self.sub_l_0 = rospy.Subscriber("/tb3_0/scan", LaserScan, self.scan_callback, queue_size=1)
        self.pub_H = rospy.Publisher("/tb3_0/Histogram", Histogram, queue_size=10)
        self.r = 0.3
        self.s = 0.3
        self.alfa = 4 #tamano del sector 4 grados.
        self.a = 5
        self.b = 1
        self.H = np.zeros(90)
        self.Hp = list()
    #def steeringCallback(self,data):
    #    if self.Hp[int(data.steering/4)] < 1:
    #        twist = Twist()
    #        twist.linear.x = BURGER_MAX_LIN_VEL; twist.linear.y = 0.0; twist.linear.z = 0.0
    #        #print(twist.linear.x)
    #        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = data.steering*Kp
    #        self.pub.publish(twist)
    #    else:
    #        for k in range(90):
    #            if self.Hp[k] < 1:
    #                if k == 0:
    #                    gmin = 5*min(k,int(data.steering/4)) + 2*min(k,int(data.yaw/4))
    #                    gpast = gmin
    #                    orientation = k
    #                else:
    #                    gmin = 5*min(k,int(data.steering/4)) + 2*min(k,int(data.yaw/4))
    #                    if gmin < gpast:
    #                        gpast = gmin
    #                        orientation = k

    def scan_callback(self,data):
        # Guardar datos en histograma
        #print(data.ranges)
        self.H = np.zeros(90)#Crear vector de 90 elementos
        size = np.size(data.ranges) #Obtiene el tamano de los datos (360)
        for beta in range(size): #For hasta 360
            #print(data.ranges[beta])
            if data.ranges[beta] > 2: #Si la distancia es mayor a 2
                d = 0 #d=0
                #print(beta, d)
            else:
                d = data.ranges[beta] #Si no guarda la distancia
                #print(beta, d)
                k = int((beta)/self.alfa) # k_alfa es el sector actualmente en calculo
                if beta<120 or (beta>240 and beta<360):
                    #if beta>(beta - np.arcsin((self.r + self.s)/d)) and beta<(beta + np.arcsin((self.r + self.s)/d)):
                    previus = self.H[k]
                    self.H[k]=(previus + (15*(self.a-self.b*d*d)))
        msg_to_send = Histogram()
        msg_to_send.Histogram = self.H
        self.pub_H.publish(msg_to_send)





def main():
    try:
        rospy.init_node('LaseSub')
        LaserSub()  # constructor creates publishers / subscribers
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()
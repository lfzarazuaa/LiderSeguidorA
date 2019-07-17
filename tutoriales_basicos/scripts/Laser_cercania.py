#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Zarazua #

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Obstacle():
    def __init__(self):
        self.LIDAR_ERR = 0.05 #Limite para determinar si valida la lectura o no. 
        self.dist_min = 0.4 #Distancia minima.
        self.obstacle() #Ejecuta funcion de deteccion de obstaculos
        
    def get_scan(self):
        msg = rospy.wait_for_message("/scan", LaserScan) #Subscriptor de una sola vez.
        self.scan_filter = [] #Crea lista vacia.
        self.scan_filter_err = []
        for i in range(360): #For para cada punto del lidar.
            if i <= 135 or i > 225: #Obtiene las distancias de los angulos que se quieran.
                if msg.ranges[i] >= self.LIDAR_ERR: #Valida si es una distancia no erronea.
                    self.scan_filter.append(msg.ranges[i]) #Guarda todos los angulos.
                else:
                    self.scan_filter_err.append(msg.ranges[i])
        print 'distancias erroneas=', self.scan_filter_err
                    
    def obstacle(self):
        while not rospy.is_shutdown():
            self.get_scan() #Obtiene las distancias del lidar.
            dist=min(self.scan_filter)
            if dist < self.dist_min:#Verifica que sean menores a la distancia minima
                rospy.loginfo('Parar! Distancia al obstaculo : %f', dist)
            else:
                rospy.loginfo('Distancia al obstaculo : %f', dist)

def main():
    rospy.init_node('turtlebot3_obstacle')
    print 'Nodo Inicializado'
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

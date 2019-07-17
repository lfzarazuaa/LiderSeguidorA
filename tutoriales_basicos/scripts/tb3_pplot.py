#!/usr/bin/env python
from multiprocessing import Process
import sys
import rospy
import numpy as np
import os
import time
import matplotlib.pyplot as plt
import pandas as pd
from geometry_msgs.msg import PoseWithCovarianceStamped
from turtlesim.msg import Pose
from scipy.spatial import KDTree
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64,Int32,Bool
plotear=True
opcion=1
m=2#Plotter
n=1#Plotter
p=20#Numero de puntos del arbol entre 2 posiciones consecutivas en la trayectoria 
class Plotter:
    def __init__(self):
      self.poseflag=False
      ruta=os.path.dirname(os.path.abspath(__file__))+'/Generacion _de_Trayectorias/'
      PuntosA=np.array(np.load(ruta+'PuntosA1.npy'))
      self.ArbolA=self.Arbol(PuntosA,p)
      PuntosB=np.array(np.load(ruta+'PuntosB1.npy'))
      self.ArbolB=self.Arbol(PuntosB,p)
      PuntosC=np.array(np.load(ruta+'PuntosC1.npy'))
      self.ArbolC=self.Arbol(PuntosC,p)
      self.turtlebot3_pose_L=Pose()
      self.Positions_XL=[]
      self.Positions_YL=[]
      self.Positions_EL=[]
      self.turtlebot3_pose_F=Pose()
      self.Positions_XF=[]
      self.Positions_YF=[]
      self.Positions_EF=[]
      self.Positions_Count=[]
      self.count=0
      self.lane=0
      self.graph=False
      opcion=1#Accion a graficar incialmente
      self.p = Process(None, self.graficas)
      self.p.start()
      #self.fig = plt.figure(figsize=(7,7), facecolor='w')
      #self.fig.canvas.set_window_title('Trayectorias generadas')
      self.lane_subscriber=rospy.Subscriber("/lane",Int32,self.laneCallback,queue_size=1)
      #self.posel_subscriber=rospy.Subscriber("/tb3_0/amcl_pose", PoseWithCovarianceStamped,self.poseCallback,queue_size=1)
      #self.posef_subscriber=rospy.Subscriber("/tb3_1/amcl_pose", PoseWithCovarianceStamped,self.poseCallback2,queue_size=1)
      self.graph_subscriber=rospy.Subscriber("/graph", Bool,self.graph_on_Callback,queue_size=1)
      self.clear_graph_subscriber=rospy.Subscriber("/clear_graph",Bool,self.clear_graph_Callback,queue_size=1)
      self.save_data_subscriber=rospy.Subscriber("/save_data",Bool,self.save_data_Callback,queue_size=1)
    
    def graficas(self): 
        while plotear:
            self.rate = rospy.Rate(10)
            print 'opcion=',opcion
            if opcion==1:
                self.opcion=0
                self.ion=plt.ion()
                self.fig= plt.figure()
                self.plot1 = self.fig.add_subplot(m,n,1)
                self.plot2 = self.fig.add_subplot(m,n,2)
                self.plot1.hold(True)
                self.plot1.set_title('Grafica 1')
                self.plot1.plot(1,1,'*r')
                self.plot1.plot(1.2,1.2,'*b')
                self.plot1.axis([-1.25,1.25,-1.25,1.25])
                self.plot2.hold(True)
                self.plot2.set_title('Grafica 2')
                self.plot2.plot(1,1,'*r')
                self.plot2.plot(1.2,1.2,'*b')
                self.plot2.axis([-1.25,1.25,-1.25,1.25])
                plt.pause(0.00000000001)
            elif opcion==2:
                self.opcion=0
                x=np.arange(-1.25,1.25+0.1,0.1)
                if self.graph==True:
                    #Posicion Seguidor
                    y1=np.sin(x)
                    y2=np.cos(x)
                    self.plot1.hold(True)
                    #self.plot1.set_title('Grafica 1')
                    self.plot1.plot(x,y1,'*r')
                    self.plot1.axis([-1.25,1.25,-1.25,1.25])
                    plt.pause(0.00000000001)
                    ##
                    self.plot2.hold(True)
                    #self.plot2.set_title('Grafica 2')
                    self.plot2.plot(x,y2,'*r')
                    self.plot2.axis([-1.25,1.25,-1.25,1.25])
                    plt.pause(0.00000000001)
                else:
                    #Posicion Seguidor
                    y1=-np.sin(x)
                    y2=-np.cos(x)
                    self.plot1.hold(True)
                    #self.plot1.set_title('Grafica 1')
                    self.plot1.plot(x,y1,'*r')
                    self.plot1.axis([-1.25,1.25,-1.25,1.25])
                    plt.pause(0.00000000001)
                    ##
                    self.plot2.hold(True)
                    #self.plot2.set_title('Grafica 2')
                    self.plot2.plot(x,y2,'*r')
                    self.plot2.axis([-1.25,1.25,-1.25,1.25])
                    plt.pause(0.00000000001)
            elif opcion==3:
                self.opcion=0
                self.plot1.cla()
                self.plot1.set_title('Grafica 1')
                plt.pause(0.00000000001)
                self.plot2.cla()
                self.plot2.set_title('Grafica 2')
                plt.pause(0.00000000001)
            else:
                pass
            self.rate.sleep()

    def laneCallback(self,data):
        self.lane=data.data

    def graph_on_Callback(self,data):
        self.graph=data.data
        opcion=2
        print 'opcion puesta a 2'

    def clear_graph_Callback(self,data):
        opcion=3
        print 'opcion puesta a 3'

    def save_data_Callback(self,data):
        if self.count>2:
            po=np.array(self.Positions_Count)
            xl=np.array(self.Positions_XL)
            yl=np.array(self.Positions_YL)
            el=np.array(self.Positions_EL)
            xf=np.array(self.Positions_XF)
            yf=np.array(self.Positions_YF)
            ef=np.array(self.Positions_EF)
            matrix=np.array([po,xl,yl,el,xf,yf,ef]).T
            ruta=os.path.dirname(os.path.abspath(__file__))
            nombre_Archivo='/Datos_Guardados/Grafica_'+time.strftime("%Y_%m_%d_%H_%M_%S")
            nombre_ArchivoP= nombre_Archivo+'.npy'
            nombre_ArchivoE= nombre_Archivo+'.xlsx'
            np.save(ruta+nombre_ArchivoP, matrix)
            df = pd.DataFrame(matrix,columns = ['Muestra','x Lider','y Lider','error Lider','x Seguidor','y Seguidor','error Seguidor'])
            df.to_excel(ruta+nombre_ArchivoE, sheet_name='Datos Obtenidos')

    def Arbol(self,xy,longitud):
        ax,ay=xy.T
        l=len(ax)
        x=[]
        y=[]
        for i in range(l+1):
            ind1=i%l
            ind2=(i+1)%l
            xo=ax[ind1]
            xf=ax[ind2]
            diferenciaX=xf-xo
            yo=ay[ind1]
            yf=ay[ind2]
            diferenciaY=yf-yo
            for j in range(longitud):
                escala=float(j)/float(longitud)
                x.append(xo+diferenciaX*escala)
                y.append(yo+diferenciaY*escala)
        x1=np.array(x)
        y1=np.array(y)
        xy=np.array([x1,y1]).T
        return KDTree(xy)               
    
def main():
        rospy.init_node('Plotter', anonymous=True)
        P=Plotter() # constructor creates publishers / subscribers
        print 'PLotter inicializado'
        rospy.spin()
        plotear=False
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

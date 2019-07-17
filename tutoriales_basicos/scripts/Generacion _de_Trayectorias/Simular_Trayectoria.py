#!/usr/bin/env python2
import numpy as np
import path_parser
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.spatial import KDTree

map_size_x=250.0 #cm
map_size_y=250.0 #cm
resolution = 1.0 #cm

def main():
    #matrix = np.load('/home/zarazua_rt/catkin_ws/src/tutoriales_basicos/scripts/TrayB1.npy')
    matrix = np.load('TrayD1.npy')
    final=175
    div=1
    x1=0
    y1=0
    #ax = plt.axes()
    x=np.zeros(final,dtype='f' )
    y=np.zeros(final,dtype='f' )
    for xi in range(0, final):
        initial_position=[x1,y1]
        pos_x=x1+1.25
        pos_y=y1+1.25
        x_index=np.int(pos_x*(100/resolution))
        y_index=np.int(pos_y*(100/resolution))
        if (x_index<0):
            x_index = 0
        elif (x_index>((map_size_x/resolution)-1)):
            x_index=(map_size_x/resolution)-1
        if (y_index<0):
            y_index = 0
        elif (y_index>((map_size_y/resolution)-1)):
            y_index=(map_size_y/resolution)-1
        x2, y2 = matrix[x_index,y_index,:]
        #print x1,y1,x1+x2,y1+y2
        x[xi]=float(x1)
        y[xi]=float(y1)
        #ax.arrow(x1, y1, x2, y2 , head_width=0.01, head_length=0.01, fc='k', ec='k')
        x1=x1+x2/div
        y1=y1+y2/div
    #print x
    #print y
    plt.plot(x,y)
    plt.show()

if __name__ == '__main__':
    main()

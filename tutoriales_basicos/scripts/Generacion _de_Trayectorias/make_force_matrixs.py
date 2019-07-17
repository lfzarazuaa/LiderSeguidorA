#!/usr/bin/env python2
import numpy as np

import path_parser

import matplotlib.pyplot as plt

from scipy.spatial import KDTree

map_size_x=500 #cm
map_size_y=400 #cm
resolution = 10 #cm
lookahead_offset = 5 #*10cm
matrix = np.zeros( (map_size_x/resolution,map_size_y/resolution,2),dtype='f' )

def main(map_file):
    xy = np.multiply(np.array([0.833,0.75]),np.array(list(path_parser.read_points(map_file))))
    print(xy)
    x, y = xy.T
    plt.plot(x, y)
    tree = KDTree(xy)

    fig = plt.figure(figsize=(12, 10), facecolor='w')
    plt.plot(x, y, ':o', markersize=2)

    plt.tight_layout()#Ajusta los titulos de subplots para evitar que salgan de la figura.

    def show_nearest(target):
        dist, index = tree.query(target) #Obtiene los puntos mas cercanos al camino
        global lookahead_offset
        lookahead_offset = np.int(2 + (5/(5*dist+1)))
        lookahead_target = xy[(index + lookahead_offset) % len(xy)]

        x1, y1 = target
        x3, y3 = lookahead_target

        plt.scatter(*target, color='r')
        plt.scatter(*xy[index], color='g')
        ax = plt.axes()
        ax.arrow(x1, y1, (x3-x1)/5, (y3-y1)/5 , head_width=0.01, head_length=0.01, fc='k', ec='k')
        plt.scatter(*lookahead_target, color='m')
        plt.show(block=False)
        global matrix
        x_index=np.int(x1*10)
        y_index=np.int(y1*10)

        matrix[x_index,y_index,0]=x3-x1
        matrix[x_index,y_index,1]=y3-y1

    print('please wait ...')
    for x in range(0, map_size_x/resolution):
        for y in range(0, map_size_y/resolution):
            show_nearest((x*5*resolution/map_size_x, y*4*resolution/map_size_y))


    #np.save('matrixDynamic_tt_1.npy', matrix)
    #print('matrix is saved.')
    plt.show()
 



if __name__ == '__main__':
    main('sample_map_origin_map.txt')

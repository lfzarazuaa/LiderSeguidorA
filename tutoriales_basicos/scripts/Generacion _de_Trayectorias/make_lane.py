#!/usr/bin/env python2
import numpy as np
import path_parser
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.spatial import KDTree
#ruta='sample_map_origin_map_1.txt'
ruta='Trayectoria2.txt'
nombre='TrayB1.npy'
map_size_x=250.0 #cm
map_size_y=250.0 #cm
resolution = 1.0 #cm
matrix = np.zeros( (map_size_x/resolution,map_size_y/resolution,2),dtype='f' )
matrix_dist = np.zeros( (map_size_x/resolution,map_size_y/resolution),dtype='f' )

def show_nearest(target,tree,xy):
        dist, index = tree.query(target) #Obtiene los puntos mas cercanos al camino
        global lookahead_offset
        lookahead_offset = np.int(1 + (5/(5*dist+1)))
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

def near(initial_position,xind,yind,tree,xy,ax):
         dist, index = tree.query(initial_position)
         global matrix_dist
         matrix_dist[xind,yind]=dist
         #Encontar el punto mas cercano a llegar
         lookahead_offset = np.int(2 + (5/(5*dist+1)))
         lookahead_target = xy[(index + lookahead_offset) % len(xy)]
         x1, y1 = initial_position
         x3, y3 = lookahead_target
         #print x1,y1,x3,y3
         #plt.scatter(*initial_position, color='r')
         #plt.scatter(*xy[index], color='g')
         #ax.arrow(x1, y1, (x3-x1), (y3-y1) , head_width=0.01, head_length=0.01, fc='k', ec='k')
         #plt.scatter(*lookahead_target, color='m')
         x_index=xind
         y_index=yind
         matrix[x_index,y_index,0]=x3-x1 #distancias en x
         matrix[x_index,y_index,1]=y3-y1 #distancias en y
def main():
    arr_in=np.array(list(path_parser.read_points(ruta)))
    ld=0.75
    min_dist=0.025
    ax,ay=arr_in.T
    min_x=np.min(ax)
    min_y=np.min(ay)
    max_x=np.max(ax)
    max_y=np.max(ay)
    print 'Minimo en x',min_x
    print 'Minimo en y',min_y
    print 'Maximo en x',max_x
    print 'Maximo en y',max_y
    lenx=(max_x-min_x)
    leny=(max_y-min_y)
    offsetx=-(max_x-lenx/2)
    offsety=-(max_y-leny/2)
    arr_in = arr_in+np.array([offsetx,offsety])
    #Escalar
    ax,ay=arr_in.T
    min_x=np.min(ax)
    min_y=np.min(ay)
    max_x=np.max(ax)
    max_y=np.max(ay)
    print 'Minimo en x',min_x
    print 'Minimo en y',min_y
    print 'Maximo en x',max_x
    print 'Maximo en y',max_y
    lenx=(max_x-min_x)
    leny=(max_y-min_y)
    scale_x=2*ld/(lenx)
    scale_y=2*ld/(leny)
    scale=np.array([scale_x,scale_y])
    xy = np.multiply(scale,arr_in)+np.array([(map_size_x/2)/100,(map_size_y/2)/100])
    print xy
    x,y = xy.T
    l=len(x)
    xo=x[0]
    yo=y[0]
    xl=[xo]
    yl=[yo]
    for i in range(l):
            xf=x[i]
            yf=y[i]
            dist=np.sqrt((xf-xo)**2+(yf-yo)**2)
            if dist>min_dist:
                    xo=xf
                    yo=yf
                    xl.append(xf)
                    yl.append(yf)
    x1=np.array(xl)
    y1=np.array(yl)
    xy=np.array([x1,y1]).T
    print xy
    fig = plt.figure(figsize=(7,7), facecolor='w')
    fig.canvas.set_window_title('Trayectoria')
    plt.plot(x1-(map_size_x/2)/100,y1-(map_size_y/2)/100)
    plt.plot(x1-(map_size_x/2)/100,y1-(map_size_y/2)/100, ':o', markersize=4)
    plt.tight_layout()#Ajusta los titulos de subplots para evitar que salgan de la figura. np.array([(map_size_x/2)/100,(map_size_y/2)/100])
    plt.axis([-1,1,-1,1])
    plt.show()
    tree = KDTree(xy)

    print('please wait ...')
    X=np.arange(0,map_size_x/100,resolution/100)
    Y=np.arange(0,map_size_y/100,resolution/100)
    X,Y=np.meshgrid(X,Y)
    lim_x=int(map_size_x/resolution);
    lim_y=int(map_size_y/resolution);
    print lim_x,lim_y
    fig = plt.figure(figsize=(7,7), facecolor='w')
    fig.canvas.set_window_title('Puntos de prueba')
    ax = plt.axes()
    for xi in range(0, lim_x):
        print float(xi)/lim_x*100
        for yi in range(0, lim_y):
            #show_nearest((x,y))
            near((xi*resolution/100,yi*resolution/100),xi,yi,tree,xy,ax)
    Z=matrix_dist;
    fig = plt.figure(figsize=(7,7), facecolor='w')
    ax = fig.gca(projection='3d')
    fig.canvas.set_window_title('Distancias')
    surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,linewidth=0, antialiased=False)
    plt.show()
    np.save(nombre, matrix)
    print('matrixForce is saved.')

if __name__ == '__main__':
    main()
    cadena='Hola'

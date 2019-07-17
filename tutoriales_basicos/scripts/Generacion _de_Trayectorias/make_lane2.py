#!/usr/bin/env python2
import numpy as np
import os
import path_parser
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.spatial import KDTree

#ruta='sample_map_origin_map_1.txt'
puntos='/Trayectoria3.txt'
#nombre='/home/zarazua_rt/catkin_ws/src/tutoriales_basicos/scripts/TrayD1.npy'
nombre='/TrayC1.npy'
nombrep='/PuntosC1.npy'
map_size_x=250.0 #cm
map_size_y=250.0 #cm
resolution = 1.0 #cm
matrix = np.zeros( (map_size_x/resolution,map_size_y/resolution,2),dtype='f' )
matrix_dist = np.zeros( (map_size_x/resolution,map_size_y/resolution),dtype='f' )
ind_lookahead = np.zeros( (map_size_x/resolution,map_size_y/resolution),dtype='d' )
ld=0.75#Longitud desde la mitad del mapa.
min_dist=0.05#Distancia minima de separacion entre 2 puntos.
def near(initial_position,xind,yind,tree,xy,ax):
         dist, index = tree.query(initial_position)
         global matrix_dist
         matrix_dist[xind,yind]=dist
         #print(dist)
         #Encontar el punto mas cercano a llegar
         #lookahead_offset = np.int(1 + (5/(0.25*dist+1)))
         if dist>4*min_dist:
             lookahead_offset = np.int(0)
         else:
             lookahead_offset = np.int(2)
         ind_lookahead[xind,yind]=lookahead_offset
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

def Arbol(xy,longitud):
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
    print 'Separacion'
    print np.array([x1,y1])
    fig2 = plt.figure(figsize=(7,7), facecolor='w')
    fig2.canvas.set_window_title('Extrapolacion')
    plt.plot(x1,y1,'*b')
    plt.axis([-1,1,-1,1])
    plt.show()
    return KDTree(xy)

def main():
    ruta=os.path.dirname(os.path.abspath(__file__))
    arr_in=np.array(list(path_parser.read_points(ruta+puntos)))
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
    dist=np.sqrt((x[0]-xf)**2+(y[0]-yf)**2)
    print dist
    if dist<min_dist:
        xl.pop()
        yl.pop()
    x1=np.array(xl)
    y1=np.array(yl)
    xy=np.array([x1,y1]).T
    xyA=np.array([x1-(map_size_x/2)/100,y1-(map_size_y/2)/100]).T
    np.save(ruta+nombrep, xyA)
    print('points are saved')
    print xy
    fig = plt.figure(figsize=(7,7), facecolor='w')
    fig.canvas.set_window_title('Trayectoria')
    plt.plot(x1-(map_size_x/2)/100,y1-(map_size_y/2)/100)
    l=len(x1)-1
    plt.plot(np.array([x1[0],x1[l]])-(map_size_x/2)/100,np.array([y1[0],y1[l]])-(map_size_y/2)/100)
    plt.plot(x1-(map_size_x/2)/100,y1-(map_size_y/2)/100, ':o', markersize=4)
    plt.tight_layout()#Ajusta los titulos de subplots para evitar que salgan de la figura. np.array([(map_size_x/2)/100,(map_size_y/2)/100])
    plt.axis([-1,1,-1,1])
    plt.show()
    tree = KDTree(xy)
    tree2= Arbol(xyA,30)

    # print('please wait ...')
    # X=np.arange(0,map_size_x/100,resolution/100)
    # Y=np.arange(0,map_size_y/100,resolution/100)
    # X,Y=np.meshgrid(X,Y)
    # lim_x=int(map_size_x/resolution);
    # lim_y=int(map_size_y/resolution);
    # print lim_x,lim_y
    # fig = plt.figure(figsize=(7,7), facecolor='w')
    # fig.canvas.set_window_title('Puntos de prueba')
    # ax = plt.axes()
    # for xi in range(0, lim_x):
    #     print float(xi)/lim_x*100
    #     for yi in range(0, lim_y):
    #         #show_nearest((x,y))
    #         near((xi*resolution/100,yi*resolution/100),xi,yi,tree,xy,ax)
    # Z=matrix_dist;
    # dist_min=Z.min()
    # dist_max=Z.max()
    # print 'dist_max=', dist_max
    # print 'dist_min=', dist_min
    # fig = plt.figure(figsize=(7,7), facecolor='w')
    # ax = fig.gca(projection='3d')
    # fig.canvas.set_window_title('Distancias')
    # surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,linewidth=0, antialiased=False)
    # plt.show()
    # np.save(nombre, matrix)
    # print('matrixForce is saved.')


if __name__ == '__main__':
    main()
    cadena='Hola'

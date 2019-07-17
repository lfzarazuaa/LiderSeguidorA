#!/usr/bin/env python2
import numpy as np
import path_parser
import matplotlib.pyplot as plt

def GuardarArchivo(nombre):
    xr,yr=puntos
    npuntos=len(xr)
    archivo=open(nombre+'.txt','w')
    archivo.write('/* Sample RNDF change log*/\n')
    archivo.write('/*                                                 */\n')
    archivo.write('/* Mar 21, 2019 - RNDF - Anticlockwise road selection*/\n')
    archivo.write('/*                                                 */\n')
    archivo.write('/*                                                 */\n')    
    archivo.write('RNDF_name RNDF_Rev_1.0\n')
    archivo.write('num_segments 1\n')
    archivo.write('num_zones 0\n')
    archivo.write('format_version 1.0\n')
    archivo.write('creation_date 21-Mar-19\n')
    archivo.write('segment 1\n')
    archivo.write('num_lanes 1\n')
    archivo.write('segment_name Outer_Loop\n')
    archivo.write('lane 1.2\n')
    archivo.write('num_waypoints '+ str(npuntos) + '\n')
    archivo.write('lane_width 0.35\n')
    archivo.write('left_boundary solid_white\n')
    archivo.write('right_boundary broken_white\n')
    for i in range(1,npuntos+1):
        archivo.write('1.2.' + str(i) + '	' + str(xr[i-1]) +'	'+ str(yr[i-1]) + '\n')
    archivo.close()
def cvector(inicio,final,paso):
    return np.arange(inicio,final+paso,paso)
def main():
    m,n=(1,2);
    th=np.arange(0,np.pi/2,1*np.pi/180);
    r=np.sin(2*th)
    rl=[]
    tl=[]
    for i in range(0,len(r)):
        if abs(r[i])>0.35:
            rl.append(r[i])
            tl.append(th[i])
    
    xn=np.array(rl)*np.cos(np.array(tl))
    yn=np.array(rl)*np.sin(np.array(tl))
    x=np.concatenate((xn,-xn[::-1],-xn,xn[::-1]))+1.25
    y=np.concatenate((yn,yn[::-1],-yn,-yn[::-1]))+1.25
    plt.ioff()#Desactivar modo interactivo
    plt.subplot(m,n,1)
    plt.title('Puntos de Trayectoria')
    plt.plot(x,y)
    plt.subplot(m,n,2)
    plt.title('Trayectoria cerrada')
    plt.scatter(x,y)
    plt.show()
    global puntos
    puntos=(x,y)
    GuardarArchivo('Trayectoria4');

if __name__ == '__main__':
    main()
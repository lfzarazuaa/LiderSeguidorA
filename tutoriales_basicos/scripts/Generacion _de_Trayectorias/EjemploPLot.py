#!/usr/bin/env python2
import numpy as np
import path_parser
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
def cvector(inicio,final,paso):
    return np.arange(inicio,final+paso,paso)
def main():
    m=1
    n=2
    x=np.arange(0,10,0.1);
    y=x**2/100
    plt.ioff()#Activar modo interactivo
    plt.figure('Figura1')
    plt.figure('Figura2')
    a=np.random.rand(100)
    b=np.random.rand(100)
    plt.figure('Figura1')
    plt.subplot(m,n,1)
    y1=cvector(1,5,0.1)
    print y1
    plt.plot(y1)
    plt.subplot(m,n,2)
    y2=cvector(5,1,-0.1)
    plt.plot(y2)
    print y2
    plt.figure('Figura2')
    plt.plot(a,b)
    plt.scatter(x,y)
    plt.figure('Figura3')
    m=2
    n=2
    plt.subplot(m,n,1)
    y=cvector(1,3,0.1)
    plt.plot(y)
    plt.subplot(m,n,2)
    y=cvector(3,1,-0.1)
    plt.plot(y)
    plt.subplot(2,1,2)
    y1=np.array(cvector(0,2,0.1))
    y2=np.array(cvector(2,0,-0.1))
    y=np.concatenate((y1,y2),axis=0)
    #y=np.array([0,1,2,2,1,0])
    plt.plot(y)
    fig = plt.figure(frameon=True,edgecolor='b',dpi=100,figsize=(12, 12), facecolor='g')
    fig.canvas.set_window_title('Figura 5')
    #print id(fig)
    #fig.figure('Figura4')
    x=np.array(cvector(0,10,0.1))
    y=np.array(cvector(0,10,0.1))
    l=len(x)
    print x
    for i in range(0,l):
      if x[i]>5:
          y[i]=2    
      else:
          y[i]=0
    plt.plot(x,y)
    # plt.plot(np.random.rand(10))
    # plt.plot(np.random.rand(10))
    #plt.clf()
    plt.show()
    print plt.isinteractive()
    print plt.ishold()

if __name__ == '__main__':
    main()
    cadena='Hola'
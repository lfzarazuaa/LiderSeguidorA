#!/usr/bin/env python2
import numpy as np
import path_parser
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.spatial import KDTree
#ruta='sample_map_origin_map.txt'
ruta='Trayectoria3.txt'

def main():
    arr_in=np.array(list(path_parser.read_points(ruta)))
    print arr_in
if __name__ == '__main__':
    main()
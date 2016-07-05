# -*- coding: utf-8 -*-

from tsp3D_class import Tsp3D
import matplotlib.pyplot as plt
import numpy as np
import random as random

if __name__ == '__main__':
    # Example implementation of the genetic TSP algorithm
    if 1:
        N = 15
        x = np.array([random.randint(0,N) for ii in range(N)])
        y = np.array([random.randint(0,N) for ii in range(N)])
        z = np.array([random.randint(0,N) for ii in range(N)])
        tsp = Tsp3D(x, y, z, iterationLim=100)
        tsp()
        tsp.plot()

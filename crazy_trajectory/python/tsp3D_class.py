# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
import random
import copy as copy
from mpl_toolkits.mplot3d import Axes3D

class Tsp3D(object):
    
    def __init__(self, x, y, z, populationSize=100, iterationLim=40):
        """
        Create a solver object for generating and visualizing a close to
        optimal solution to the 3D TSP problem.
        
        ARGS:
            x (numpy array): A 1xN array of x-points.
            y (numpy array): A 1xN array of y-points.
            z (numpy array): A 1xN array of z-points.
            populationSize (int): The number of solutions in the population.
            iterationLim (int): The maximum number of allowed iterations.
        RETURNS:
            None
        """
        self.data = np.array([x, y, z])
        self.populationSize = populationSize
        self.iterationLim = iterationLim
        self.solutionHistory = []
        self.path = []
        self.dist = self._distance_matrix()

    def plot(self):
        """
        Plots the solution as an interactive 3D-plot, as well as the distance
        matrix and the solution history (in terms of total loop distance).
        ARGS:
            None.
        RETURNS:
            None.
        """
        plt.figure()
        ax = plt.subplot2grid((2,2), (0,0), colspan=2, projection='3d')
        for ii in range(len(self.path) - 1):
            ax.plot([self.data[0][self.path[ii]], self.data[0][self.path[ii + 1]]],
                    [self.data[1][self.path[ii]], self.data[1][self.path[ii + 1]]],
                    [self.data[2][self.path[ii]], self.data[2][self.path[ii + 1]]], 'k')

        ax.plot(self.data[0], self.data[1], self.data[2],'ro', markersize=5)
        plt.xlabel('x [m]'), plt.ylabel('y [m]'), plt.ylabel('z [m]')
        plt.title('Shortest 3D distance (black) connecting all points (red)\n')
        
        ax = plt.subplot2grid((2,2), (1,0))
        plt.title('Distance matrix (point i to point j)')
        ax.matshow(self.dist)
        
        ax = plt.subplot2grid((2,2), (1,1))
        ax.plot([ii for ii in range(len(self.solutionHistory))], self.solutionHistory)
        plt.title('Distance as a function of number of\niterations (or generations)')
        plt.ylabel('Distance [m]'), plt.xlabel('Number of iterations')
        plt.show()

    def _distance_matrix(self):
        """
        Generates a distance matrix for the specified data.
        ARGS:
            None
        RETURNS:
            dist (numpy array): An NxN symmetrical matrix where N is the number
                of 3D points in the data.
        """
        def dist(ii, jj):
            """
            Calculates a distance between two points at indices ii and jj in
            the xy data matrix.
            ARGS:
                ii, jj (int): Indices
            """
            return (sqrt((self.data[0][ii] - self.data[0][jj]) ** 2 +
                         (self.data[1][ii] - self.data[1][jj]) ** 2 +
                         (self.data[2][ii] - self.data[2][jj]) ** 2))
        return np.array([np.array([dist(ii, jj) for jj in range(len(self.data[0]))]) for ii in range(len(self.data[0]))])
    
    def __call__(self):
        # Generates a population
        n = len(self.data[0])
        population = np.array([[ii for ii in range(n)] for jj in range(self.populationSize)])
        for ii in range(1, self.populationSize):
            random.shuffle(population[ii])

        for iteration in range(self.iterationLim):
            # Computes the total distance for each population member
            populationDist = np.array([sum([self.dist[population[jj][ii - 1], population[jj][ii]] for ii in range(n)]) for jj in range(self.populationSize)])
            randomizedIndices = [ii for ii in range(self.populationSize)]
            random.shuffle(randomizedIndices)
            
            path, minDistance = self._bestSolution(population)
            self.path = path + [path[0]]
            self.solutionHistory.append([minDistance])
        
            newPopulation = []
            for ii in range(self.populationSize // 4):
                selectedPopulations = population[randomizedIndices[4 * ii : 4 * (ii + 1)]]
                selectedDistances = populationDist[randomizedIndices[4 * ii : 4 * (ii + 1)]]
                index = np.where(selectedDistances == selectedDistances.min())[0][0]
                bestRoute = selectedPopulations[index]
                breakPoints = [random.randint(0, n - 1), random.randint(0, n - 1)]
                breakPoints.sort(key=int)

                offspring = copy.copy(bestRoute)
                    
                flip = copy.copy(bestRoute).tolist()
                flipSection = flip[breakPoints[0]:breakPoints[1]]
                flipSection.reverse()
                flip = flip[:breakPoints[0]] + flipSection + flip[breakPoints[1]:]
                offspring = np.append([offspring], [flip], axis=0)
                         
                swap = copy.copy(bestRoute)
                swap[breakPoints[0]], swap[breakPoints[1]] = swap[breakPoints[1]], swap[breakPoints[0]]
                offspring = np.append(offspring, [swap], axis=0)
            
                slide = copy.copy(bestRoute).tolist()
                poppedElement = slide.pop(breakPoints[1])
                slide.insert(breakPoints[0], poppedElement)
                offspring = np.append(offspring, [slide], axis=0)
            
                if newPopulation == []:
                    newPopulation = offspring
                else:
                    newPopulation = np.append(newPopulation, offspring, axis=0)
            population = newPopulation
    
    def _bestSolution(self, population):
        """
        Compares an retreives the best solution in the population, returning
        the best solution in terms of shortest total distance as well as its
        total distance
        
        ARGS:
            population (numpy array): An Mx(N+1) matrix where each row
                constitutes a feasible path of points, from an arbitrary start
                point to finish. Each row is has a number of elements equal to
                the number of 3D points in the data plus one (as the first and
                last points will be the same) and M is the populationSize, by
                default set to 100.
        RETURNS:
            path (numpy array): An 1x(N+1) array describing the best path in
                the population.
            distance (float): The distance of the best path in the population.
        """
        populationDist = np.array([sum([self.dist[population[jj][ii - 1], population[jj][ii]] for ii in range(len(population[0]))]) for jj in range(self.populationSize)])
        index = np.where(populationDist == populationDist.min())
        return population[index[0][0]].tolist(), populationDist.min()


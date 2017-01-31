"""
Demonstrates the visualization saving and loading of the trajectories
"""
import os,sys,inspect
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
modulesdir = os.path.dirname(curdir)
rootdir = os.path.dirname(modulesdir)
sys.path.insert(0,modulesdir)

from trajectorylib import Trajectory

if __name__ == "__main__":
    directory = os.path.join(rootdir, 'trajectories')
    fnname = 'p_1'
    fformat = 'json'
    t = Trajectory(directory, fnname, fformat)
    t.visualize_fixed(['x','y','z'], ['1D', '2D', '3D'])
    t.save('p_1.json')

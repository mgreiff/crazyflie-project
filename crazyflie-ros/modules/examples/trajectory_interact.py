"""This example demonstrates the trajectory interaction by loading a trajectory
allowing the user to interact with it.
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
    t.visualize_interactive('x')
    
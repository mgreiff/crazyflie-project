# -*- coding: utf-8 -*-

import os,sys,inspect

# Loads crazylib from the parent directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
pardir = os.path.dirname(curdir)
sys.path.insert(0,pardir) 

# Imports the c_cvx_solver handle
from crazylib import c_cvx_solver

# This example calls the cavx solver in solver.c through a cython
# c-wrapper defined in crazylib and plots the result. Note that the
# system dynamics is defined in the mpc_solver.c file, and does not
# use the .cnf file in config. In order to change solver settings,
# the c-file need to be updated independently.

print c_cvx_solver([1,2,-3,4,-5,6])

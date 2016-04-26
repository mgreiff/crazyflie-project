# -*- coding: utf-8 -*-

import os,sys,inspect

# Loads crazylib from the parent directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
pardir = os.path.dirname(curdir)
sys.path.insert(0,pardir) 

import crazylib

print crazylib.c_sum([1,2,-3,4,-5,6])

# -*- coding: utf-8 -*-
import json 
import os
import sys
import numpy as np
import datetime
from math import pi

# ~~~ Data structureos definition for the trajectory packets ~~~
# Description  Path         Polynomial          Function
# Data         point (x)  | coefficient (p_0) | Amplitude (A)
# Data         point (y)  | coefficient (p_1) | Offset (B)
# Data         point (z)  | coefficient (p_2) | Frequency (F)
# Data         point (yaw)| coefficient (p_3) | Phase (P)
# Data         -          | coefficient (p_4) | -
# Data         -          | coefficient (p_5) | -
# Time            - The time during which which the trajectory is followed
# Index           - The index of the point in the sequence
# Dimension       - Set to one of {0 1 2 3} corresponding to {x,y,z,yaw}
# Trajectory type - 0 for path, 1 for poly, 2 for function, 3 for bezier
# Total (the total number of points in the sequence) 

# Whether or not the trajectory should be periodical and or if the
# start time should be zero separate is set by the parameter framework


if 1:
    # ~~~ Test sequence for evaluating flatness implementation and demonstrate file format ~~~
    
    # Offset
    xoffset = 3.0
    yoffset = 1.6
    zoffset = 1.0

    Xpolypart = [
      [     0.0+xoffset,       0.0,       0.0,    0.1304,   -0.0339,       0.0],
      [  0.5000+xoffset,    0.4785,   -0.0323,   -0.0592,    0.0091,       0.0],
      [  1.0000+xoffset,   -0.0695,   -0.1689,    0.0443,   -0.0025,       0.0],
      [  0.5000+xoffset,   -0.2930,    0.0373,   -0.0095,    0.0008,       0.0],
      [     0.0+xoffset,   -0.2318,   -0.0000,   -0.0029,   -0.0008,       0.0],
      [ -0.5000+xoffset,   -0.2930,   -0.0373,    0.0244,    0.0025,       0.0],
      [ -1.0000+xoffset,   -0.0695,    0.1689,    0.0137,   -0.0091,       0.0],
      [ -0.5000+xoffset,    0.4785,    0.0323,   -0.1411,    0.0339,       0.0]
    ];

    Ypolypart = [
      [     0.0+yoffset,       0.0,       0.0,   -0.1813,    0.0594,       0.0],
      [ -0.5000+yoffset,   -0.2748,    0.3377,   -0.0058,   -0.0159,       0.0],
      [     0.0+yoffset,    0.4967,   -0.0795,   -0.0306,    0.0043,       0.0],
      [  0.5000+yoffset,   -0.0497,   -0.1589,    0.0323,   -0.0014,       0.0],
      [     0.0+yoffset,   -0.3444,    0.0000,    0.0207,    0.0014,       0.0],
      [ -0.5000+yoffset,   -0.0497,    0.1589,    0.0041,   -0.0043,       0.0],
      [     0.0+yoffset,    0.4967,    0.0795,   -0.1333,    0.0159,       0.0],
      [  0.5000+yoffset,   -0.2748,   -0.3377,    0.2939,   -0.0594,       0.0]
    ];
     
    Zpolypart = [
      [     0.0+zoffset,       0.0,       0.0,   -0.0172,    0.0086,       0.0],
      [     0.0+zoffset,    0.0687,    0.1030,   -0.0328,   -0.0023,       0.0],
      [  0.2500+zoffset,    0.0133,   -0.1491,    0.0388,    0.0006,       0.0],
      [     0.0+zoffset,   -0.0985,    0.0980,   -0.0241,   -0.0001,       0.0],
      [     0.0+zoffset,   -0.0000,   -0.0497,    0.0251,   -0.0001,       0.0],
      [     0.0+zoffset,    0.0985,    0.0980,   -0.0436,    0.0006,       0.0],
      [  0.2500+zoffset,   -0.0133,   -0.1491,    0.0512,   -0.0023,       0.0],
      [     0.0+zoffset,   -0.0687,    0.1030,   -0.0515,    0.0086,       0.0]
    ];

    times = [5.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,3.0]
    totaltime = 0
    for time in times:
        totaltime += time
    

    # Creates structure
    numberOfEntries = 10
    now = datetime.datetime.now()
    trajectory = {'packets':[],
        'settings':{
            'circular':[0,0,0,0],                # We assume that every blender trajectrory is non-circular
            'number':[10, 10, 10, 1],
        'info':'Generated on %s'%(now.strftime("%A %d. %B %Y"))
        }
    }
    
    # Moves quadcopter to starting point over 5 seconds
    trajectory['packets'].append({'data':[xoffset, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[0], 'type': 0, 'index': 0,
                                  'number':numberOfEntries, 'dimension':0})
    trajectory['packets'].append({'data':[yoffset, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[0], 'type': 0, 'index': 0,
                                  'number':numberOfEntries, 'dimension':1})
    trajectory['packets'].append({'data':[zoffset, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[0], 'type': 0, 'index': 0,
                                  'number':numberOfEntries, 'dimension':2})
    
    # Set yaw to alternate between [-0.2,0.2] with an, no phase, no offset 
    trajectory['packets'].append({'data':[zoffset, 0.2, 0.0, 0.0, 0.0, 0.0],
                                  'time' :totaltime, 'type': 2, 'index': 0,
                                  'number':1, 'dimension':3})
                     
    # Adds polynomial trajectory      
    for  ii in range(numberOfEntries - 2):
        trajectory['packets'].append({'data':Xpolypart[ii],
                                      'time' :times[ii+1], 'type': 1, 'index': ii+1,
                                      'number':numberOfEntries, 'dimension':0})
        trajectory['packets'].append({'data':Ypolypart[ii],
                                      'time' :times[ii+1], 'type': 1, 'index': ii+1,
                                      'number':numberOfEntries, 'dimension':1})
        trajectory['packets'].append({'data':Zpolypart[ii],
                                      'time' :times[ii+1], 'type': 1, 'index': ii+1,
                                      'number':numberOfEntries, 'dimension':2})
    
    # Moves quadcopter to landing point over 3 seconds
    trajectory['packets'].append({'data':[xoffset, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[-1], 'type': 0, 'index': 9,
                                  'number':numberOfEntries, 'dimension':0})
    trajectory['packets'].append({'data':[yoffset, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[-1], 'type': 0, 'index': 9,
                                  'number':numberOfEntries, 'dimension':1})
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[-1], 'type': 0, 'index': 9,
                                  'number':numberOfEntries, 'dimension':2})
    # Saves the dictionary
    filename = 'p_1.json'
    with open(filename, 'w') as trajectoryFile:
        json.dump(trajectory, trajectoryFile, separators=(',', ':'),sort_keys=True, indent=4)

if 0:
    # ~~~ Test sequence for the christmas tree demo ~~~
    numberOfQuadcopters = 4
    treeAxisCoordinates = [5.4,1.3]
    radius = 0.7
    
    offset = 0.1
    startX = 4.1
    startY = 1.4
    startZ = 0.10
    
    # Middle position
    midX = treeAxisCoordinates[0] - radius
    midY = 1.4
    midZ = [1.8,1.8,1.8,1.8]

    # Sinusoid
    amplitude = [-radius,radius,-radius,radius]
    frequency = [0.8, 1.2, 1.2, 1.6]
    periods = [3, 4, 4, 6]

    # times
    times = [5.0, 3.0, 3.0, 3.0, 5.0]
    numberOfEntries = 5

    for ii in range(numberOfQuadcopters):

        now = datetime.datetime.now()
        trajectory = {'packets':[],
            'settings':{
                'circular':[0,0,0,0],
                'number':[numberOfEntries, numberOfEntries, numberOfEntries, 1],
            'info':'Generated on %s'%(now.strftime("%A %d. %B %Y"))
            }
        }
        
        # yaw setpoint
        trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :100.0, 'type': 0, 'index': 0,
                                      'number':1, 'dimension':3})
                                      
        # startpoint
        ind = 0
        trajectory['packets'].append({'data':[startX, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':0})
        trajectory['packets'].append({'data':[startY, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':1})
        trajectory['packets'].append({'data':[startZ, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':2})
        # midpoint
        ind = 1
        trajectory['packets'].append({'data':[midX, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':0})
        trajectory['packets'].append({'data':[midY, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':1})
        trajectory['packets'].append({'data':[midZ[ii], 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':2})

        # sinusoid
        ind = 2
        A = amplitude[ii]
        w = frequency[ii]
        time = periods[ii] * 2 * pi / w;
        BX = midX + radius
        BY = midY
        trajectory['packets'].append({'data':[A, BX, w, -pi/2, 0.0, 0.0],
                                      'time' :time, 'type': 2, 'index': ind,
                                      'number':numberOfEntries, 'dimension':0})
        trajectory['packets'].append({'data':[A, BY, w, 0.0, 0.0, 0.0],
                                      'time' :time, 'type': 2, 'index': ind,
                                      'number':numberOfEntries, 'dimension':1})
        trajectory['packets'].append({'data':[midZ[ii], 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :time, 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':2})

        # midpoint
        ind = 3
        trajectory['packets'].append({'data':[midX, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':0})
        trajectory['packets'].append({'data':[midY, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':1})
        trajectory['packets'].append({'data':[midZ[ii], 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':2})

        # endpoint
        ind = 4
        trajectory['packets'].append({'data':[startX, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':0})
        trajectory['packets'].append({'data':[startY, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':1})
        trajectory['packets'].append({'data':[startZ, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :times[ind], 'type': 0, 'index': ind,
                                      'number':numberOfEntries, 'dimension':2})
        
        # Saves the dictionary
        filename = 'demo_%s.json'%(str(ii))
        with open(filename, 'w') as trajectoryFile:
            json.dump(trajectory, trajectoryFile, separators=(',', ':'),sort_keys=True, indent=4)
    
if 0:
    # ~~~ Aggressive polynomial sequence ~~~
    xoffset = 0.5
    yoffset = 0.0
    zoffset = 0.15
    PX = [[   0.0+xoffset,       0.0,       0.0,    0.4138,   -0.1143,       0.0],
          [2.0000+xoffset,    0.6137,   -1.1835,    0.7714,   -0.0016,       0.0],
          [2.2000+xoffset,    0.5545,    1.1211,   -0.9073,    0.0318,       0.0],
          [3.0000+xoffset,    0.2018,   -1.4101,    1.3339,   -0.1256,       0.0],
          [3.0000+xoffset,    0.8810,    1.8381,   -2.6921,    0.8353,       0.0],
          [3.6000+xoffset,   -0.5000,    1.0000,   -0.6667,    0.1481,       0.0]]

    PY = [[1.5000+yoffset,       0.0,       0.0,   -0.0392,    0.0157,       0.0],
          [1.5000+yoffset,    0.2448,    0.2937,   -0.1342,   -0.0043,       0.0],
          [1.9000+yoffset,    0.4124,   -0.1346,   -0.2895,    0.0118,       0.0],
          [1.9000+yoffset,   -0.6785,   -0.9327,    0.8539,   -0.0427,       0.0],
          [1.1000+yoffset,   -0.1530,    1.3727,   -1.1522,    0.2824,       0.0],
          [1.5000+yoffset,       0.0,    0.0000,   -0.0000,    0.0000,       0.0]]

    PZ = [[1.2500+zoffset,       0.0,       0.0,    0.0612,   -0.0245,       0.0],
          [1.2500+zoffset,   -0.3822,   -0.4587,    0.3396,    0.0013,       0.0],
          [0.7500+zoffset,   -0.2756,    0.5678,   -0.2932,    0.0011,       0.0],
          [0.7500+zoffset,   -0.0154,   -0.3054,    0.3264,   -0.0057,       0.0],
          [0.7500+zoffset,    0.3305,    0.6400,   -0.4088,    0.0383,       0.0],
          [1.5000+zoffset,    0.0081,   -0.6828,    0.6033,   -0.1505,       0.0]]

    times = [5.0,     2.5000,    1.0000,    1.0000,    1.0000,    1.5000,    1.5000, 5.0]
    
    # Creates structure
    numberOfEntries = 8
    now = datetime.datetime.now()
    trajectory = {'packets':[],
        'settings':{
            'circular':[0,0,0,0],                # We assume that every blender trajectrory is non-circular
            'number':[8, 8, 8, 1],
        'info':'Generated on %s'%(now.strftime("%A %d. %B %Y"))
        }
    }
    
    # Moves quadcopter to starting point over 5 seconds
    trajectory['packets'].append({'data':[PX[0][0], 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[0], 'type': 0, 'index': 0,
                                  'number':numberOfEntries, 'dimension':0})
    trajectory['packets'].append({'data':[PY[0][0], 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[0], 'type': 0, 'index': 0,
                                  'number':numberOfEntries, 'dimension':1})
    trajectory['packets'].append({'data':[PZ[0][0], 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[0], 'type': 0, 'index': 0,
                                  'number':numberOfEntries, 'dimension':2})
    
    # Set yaw to alternate between [-0.2,0.2] with an, no phase, no offset 
    trajectory['packets'].append({'data':[zoffset, 0.2, 0.0, 0.0, 0.0, 0.0],
                                  'time' :30.0, 'type': 2, 'index': 0,
                                  'number':1, 'dimension':3})
                     
    # Adds polynomial trajectory      
    for  ii in range(numberOfEntries - 2):
        trajectory['packets'].append({'data':PX[ii],
                                      'time' :times[ii+1], 'type': 1, 'index': ii+1,
                                      'number':numberOfEntries, 'dimension':0})
        trajectory['packets'].append({'data':PY[ii],
                                      'time' :times[ii+1], 'type': 1, 'index': ii+1,
                                      'number':numberOfEntries, 'dimension':1})
        trajectory['packets'].append({'data':PZ[ii],
                                      'time' :times[ii+1], 'type': 1, 'index': ii+1,
                                      'number':numberOfEntries, 'dimension':2})
    
    # Moves quadcopter to landing point over 3 seconds
    trajectory['packets'].append({'data':[PX[-1][0], 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[-1], 'type': 0, 'index': 7,
                                  'number':numberOfEntries, 'dimension':0})
    trajectory['packets'].append({'data':[PY[-1][0], 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[-1], 'type': 0, 'index': 7,
                                  'number':numberOfEntries, 'dimension':1})
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :times[-1], 'type': 0, 'index': 7,
                                  'number':numberOfEntries, 'dimension':2})
    # Saves the dictionary
    filename = 'p_2.json'
    with open(filename, 'w') as trajectoryFile:
        json.dump(trajectory, trajectoryFile, separators=(',', ':'),sort_keys=True, indent=4)
        

if 1:
    # ~~~ Steps in the z-direction ~~~
    xpos = 2.85
    ypos = 1.65
    zmax = 1.5
    zmin = 1.0
    Nz = 10
    # Creates structure
    now = datetime.datetime.now()
    trajectory = {'packets':[],
        'settings':{
            'circular':[0,0,0,0],                # We assume that every blender trajectrory is non-circular
            'number':[1, 1, Nz, 1],
        'info':'Generated on %s'%(now.strftime("%A %d. %B %Y"))
        }
    }
 
     # Moves quadcopter to starting point over 5 seconds
    trajectory['packets'].append({'data':[xpos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :3, 'type': 0, 'index': 0,
                                  'number':1, 'dimension':0})
    trajectory['packets'].append({'data':[ypos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :3, 'type': 0, 'index': 0,
                                  'number':2, 'dimension':1})
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :4, 'type': 0, 'index': 0,
                                  'number':Nz, 'dimension':2})         
    for  ii in range(4):
        trajectory['packets'].append({'data':[zmax, 0.0, 0.0, 0.0, 0.0, 0.0],              # amplitude
                                      'time' :5,                # step time
                                      'type': 0,                # LP filtering (0)
                                      'index': 2*ii+1,        # Index of trajectory spline
                                      'number':Nz,              # Total number of entries
                                      'dimension':2             # z-dimension
                                      })
        trajectory['packets'].append({'data':[zmin, 0.0, 0.0, 0.0, 0.0, 0.0],              # amplitude
                                      'time' :5,                # step time
                                      'type': 0,                # LP filtering (0)
                                      'index': 2*ii+2,        # Index of trajectory spline
                                      'number':Nz,              # Total number of entries
                                      'dimension':2             # z-dimension
                                      })
                                  
    # Set yaw to alternate between [-0.2,0.2] with an, no phase, no offset 
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :30.0, 'type': 2, 'index': 0,
                                  'number':1, 'dimension':3})
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :5, 'type': 0, 'index': 9,
                                  'number':Nz, 'dimension':2}) 
    # Saves the dictionary
    filename = 'z_1.json'
    with open(filename, 'w') as trajectoryFile:
        json.dump(trajectory, trajectoryFile, separators=(',', ':'),sort_keys=True, indent=4)

if 1:
    # ~~~ Steps in the z-direction ~~~
    xpos = 2.85
    ypos = 1.8
    zmid = 1.0
    Nz = 4
    A = 0.4
    w = 1.5
    # Creates structure
    now = datetime.datetime.now()
    trajectory = {'packets':[],
        'settings':{
            'circular':[0,0,0,0],                # We assume that every blender trajectrory is non-circular
            'number':[1, 1, Nz, 1],
        'info':'Generated on %s'%(now.strftime("%A %d. %B %Y"))
        }
    }
 
     # Moves quadcopter to starting point over 5 seconds
    trajectory['packets'].append({'data':[xpos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :3, 'type': 0, 'index': 0,
                                  'number':1, 'dimension':0})
    trajectory['packets'].append({'data':[ypos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :3, 'type': 0, 'index': 0,
                                  'number':2, 'dimension':1})
                                  
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :4, 'type': 0, 'index': 0,
                                  'number':Nz, 'dimension':2})
    trajectory['packets'].append({'data':[zmid, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :4, 'type': 0, 'index': 1,
                                  'number':Nz, 'dimension':2})
    trajectory['packets'].append({'data':[A, zmid, w, pi/2, 0.0, 0.0],
                                  'time' :30, 'type': 2, 'index': 2,
                                  'number':Nz, 'dimension':2})
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :4, 'type': 0, 'index': 3,
                                  'number':Nz, 'dimension':2})
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :30.0, 'type': 2, 'index': 0,
                                  'number':1, 'dimension':3})
    # Saves the dictionary
    filename = 'z_2.json'
    with open(filename, 'w') as trajectoryFile:
        json.dump(trajectory, trajectoryFile, separators=(',', ':'),sort_keys=True, indent=4)

if 1:
    # ~~~ Sinusoids in the xy-plane ~~~
    xpos = 2.85
    ypos = 1.65
    zpos = 1.3
    xymid = 1.0
    A = 0.5
    w = 2
    # Creates structure
    now = datetime.datetime.now()
    trajectory = {'packets':[],
        'settings':{
            'circular':[0,0,0,0],                # We assume that every blender trajectrory is non-circular
            'number':[2, 2, 3, 1],
        'info':'Generated on %s'%(now.strftime("%A %d. %B %Y"))
        }
    }
 
     # Moves quadcopter to starting point over 5 seconds
    trajectory['packets'].append({'data':[xpos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :3, 'type': 0, 'index': 0,
                                  'number':1, 'dimension':0})
    trajectory['packets'].append({'data':[A, xpos, w, 0.0, 0.0, 0.0],
                                  'time' :30, 'type': 2, 'index': 1,
                                  'number':2, 'dimension':0})
    trajectory['packets'].append({'data':[ypos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :3, 'type': 0, 'index': 0,
                                  'number':2, 'dimension':1})
    trajectory['packets'].append({'data':[A, ypos, w, pi/2, 0.0, 0.0],
                                  'time' :30, 'type': 2, 'index': 1,
                                  'number':2, 'dimension':1})
                                  
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :4, 'type': 0, 'index': 0,
                                  'number':3, 'dimension':2})
    trajectory['packets'].append({'data':[zpos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :35, 'type': 0, 'index': 1,
                                  'number':3, 'dimension':2})
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :4, 'type': 0, 'index': 2,
                                  'number':3, 'dimension':2})
                                  
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :30.0, 'type': 2, 'index': 0,
                                  'number':1, 'dimension':3})
    # Saves the dictionary
    filename = 'xy_2.json'
    with open(filename, 'w') as trajectoryFile:
        json.dump(trajectory, trajectoryFile, separators=(',', ':'),sort_keys=True, indent=4)

if 1:
    xpos = 2.85
    ypos = 1.65
    zpos = 1.3
    rad = 0.5;
    N=1+3*4
    # Creates structure
    now = datetime.datetime.now()
    trajectory = {'packets':[],
        'settings':{
            'circular':[0,0,0,0],                # We assume that every blender trajectrory is non-circular
            'number':[N, N, 3, 1],
        'info':'Generated on %s'%(now.strftime("%A %d. %B %Y"))
        }
    }
 
     # Moves quadcopter to starting point over 5 seconds
    trajectory['packets'].append({'data':[xpos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :8, 'type': 0, 'index': 0,
                                  'number':N, 'dimension':0})
    trajectory['packets'].append({'data':[ypos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :8, 'type': 0, 'index': 0,
                                  'number':N, 'dimension':1})
    for ii in range(3):
        trajectory['packets'].append({'data':[xpos - rad, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :3, 'type': 0, 'index': 4*ii+1,
                                      'number':N, 'dimension':0})
        trajectory['packets'].append({'data':[xpos - rad, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :3, 'type': 0, 'index': 4*ii+2,
                                      'number':N, 'dimension':0})
        trajectory['packets'].append({'data':[xpos + rad, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :3, 'type': 0, 'index': 4*ii+3,
                                      'number':N, 'dimension':0})
        trajectory['packets'].append({'data':[xpos + rad, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :3, 'type': 0, 'index': 4*ii+4,
                                      'number':N, 'dimension':0})
        trajectory['packets'].append({'data':[ypos + rad, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :3, 'type': 0, 'index': 4*ii+1,
                                      'number':N, 'dimension':1})
        trajectory['packets'].append({'data':[ypos - rad, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :3, 'type': 0, 'index': 4*ii+2,
                                      'number':N, 'dimension':1})
        trajectory['packets'].append({'data':[ypos - rad, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :3, 'type': 0, 'index': 4*ii+3,
                                      'number':N, 'dimension':1})
        trajectory['packets'].append({'data':[ypos + rad, 0.0, 0.0, 0.0, 0.0, 0.0],
                                      'time' :3, 'type': 0, 'index': 4*ii+4,
                                      'number':N, 'dimension':1})
                                  
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :4, 'type': 0, 'index': 0,
                                  'number':3, 'dimension':2})
    trajectory['packets'].append({'data':[zpos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :50, 'type': 0, 'index': 1,
                                  'number':3, 'dimension':2})
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :4, 'type': 0, 'index': 2,
                                  'number':3, 'dimension':2})
                                  
    trajectory['packets'].append({'data':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  'time' :30.0, 'type': 2, 'index': 0,
                                  'number':1, 'dimension':3})
    # Saves the dictionary
    filename = 'xy_1.json'
    with open(filename, 'w') as trajectoryFile:
        json.dump(trajectory, trajectoryFile, separators=(',', ':'),sort_keys=True, indent=4)

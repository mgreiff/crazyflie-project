#!/usr/bin/env python

import json

param = {'global': {
            'inner_loop_h': 0.01,    # Time discretisation in the inner loop (only used for simulation)
            'outer_loop_h': 0.1,     # Time discretisation in the inner loop (can be set < 10 Hz in PID)
            'kinect_loop_h': 0.3,    # Roughly the time between data updates from the openni
            'time_lag_receive': 0.03, # The time it takes for data sent by the quadcopter to be received at host
            'time_lag_send': 0.03    # The time it takes for data sent by the host to be received at quadcopter
        },
         'quadcopter_model': {
             'A': [0.25, 0.25, 0.25],
              'I': [0.004856, 0.004856, 0.008801],
              'Im': 3.357e-05,
              'b': 1.14e-07,
              'g': 9.81,
              'k': 2.98e-06,
              'l': 0.225,
              'm': 0.468,
              'x0': [[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.]]
        },
        'outer_PID':{
            'K': [[2.5], [1.75], [1.75], [1.75]],
            'D': [[1.5], [6.], [6.], [6.]]
        },
        'outer_MPC':{
            'Q': [[2.5], [1.75], [1.75], [1.75]],
            'R': [[1.5], [6.], [6.], [6.]]
        },
        'kinect':{
            'Q': [0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001], # Diagonal of the Q matrix
            'R': [0.1,0.1,0.1,0,0,0,0.5,0.5,0.5],                         # Diagonal of the R matrix
            'P0': [1.,1.,1.,1.,1.,1.,1.,1.,1.],                           # Diagonal of the initial covariance matrix
            'x0': [0.,0.,0.,0.,0.,0.,0.,0.,0.],                           # Initial condition for state estimation
            'measuredStates': [1,1,1,0,0,0,1,1,1]                         # Measure x,y,z-position and accelerations
        }
}

with open('configparam.cnf', 'w') as cnffile:
    json.dump(param, cnffile, separators=(',', ':'),sort_keys=True, indent=4)

#!/usr/bin/env python
import json

param = {
    'global': {
        'inner_loop_h': 0.01,    # Time discretisation in the inner loop (only used for simulation)
        'outer_loop_h': 0.1,     # Time discretisation in the inner loop (can be set < 10 Hz in PID)
        'kinect_loop_h': 0.0333, # Roughly the time between data updates from the openni
        'time_lag_receive': 0.03,# The time it takes for data sent by the quadcopter to be received
        'time_lag_send': 0.03    # The time it takes for data sent by the host to be received
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
    'PD':{
        'z':{
            'Kp':1,
            'Kd':1,
            'N':20,
            'maxlim':0.8,
            'minlim':-0.8
        }
    },
    'PID':{
        'x':{
            'Kp':1,
            'Ki':1,
            'Kd':1,
            'N':20,
            'beta':1,
            'gamma':1,
            'maxlim':0.8,
            'minlim':-0.8,
        },
        'y':{
            'Kp':1,
            'Ki':1,
            'Kd':1,
            'N':20,
            'beta':1,
            'gamma':1,
            'maxlim':0.8,
            'minlim':-0.8,
        },
        'h': 0.01
    },
    'TIMPC':{
        'Q': [10, 10, 10, 10, 10, 10, 10, 10, 10, 10],
        'R': [1, 1, 1, 1],
        'predN': 10
    },
    'TVMPC':{
        'Q': [10, 10, 10, 10, 10, 10, 10, 10, 10, 10],
        'R': [1, 1, 1, 1],
        'predN': 10
    },
    'MPC':{
        'Q': [10, 10, 10, 10, 10, 10, 10, 10, 10, 10],
        'R': [1, 1, 1, 1],
        'predN': 10
    },
    'TILQR':{
        'Kdim': (4,12),
        'K': [[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.],
              [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.],
              [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.],
              [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.]],
        'h': 0.1
    },
    'TVLQR':{
        'Kdim': (4,12),
        'K': [[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.],
              [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.],
              [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.],
              [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.]],
        'h': 0.1
    },
    'LQR':{
        'Kdim': (4,12),
        'K': [[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.],
              [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.],
              [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.],
              [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0., 0.]],
        'h': 0.1
    },
    'MAN':{
        'thrust_d': 1000,
        'pitch_d': 1,
        'roll_d': 1,
        'yaw_d': 5
    }
}

with open('configparam.cnf', 'w') as cnffile:
    json.dump(param, cnffile, separators=(',', ':'),sort_keys=True, indent=4)

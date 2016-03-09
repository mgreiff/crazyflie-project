#!/usr/bin/env python

import json

param = {'global': {
            'timestep': 0.01
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
              'x_init': [[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.]]
        },
        'inner_PD':{
            'Kd': [[2.5], [1.75], [1.75], [1.75]],
            'Kp': [[1.5], [6.], [6.], [6.]]
        }
}

with open('configparam.cnf', 'w') as cnffile:
    json.dump(param, cnffile, separators=(',', ':'),sort_keys=True, indent=4)
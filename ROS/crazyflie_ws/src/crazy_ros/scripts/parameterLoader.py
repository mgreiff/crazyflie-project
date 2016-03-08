import numpy as np
import sys
import os
import rospy
from json import dumps, load

class ParameterLoader(object):
    def load_parameters(self):
        param = None
        # Loads configuration parameters
        for filename in rospy.myargv(argv=sys.argv):
            if os.path.basename(filename) == 'configparam.cnf':
                with open(filename) as configfile:
                    param = load(configfile)
                configfile.close()
        if param is not None:
            # Sets configuration parameters
            self.timestep = param['global']['timestep']
            self.x_init = np.array(param['quadcopter_model']['x_init'])
            self.x = self.x_init

            self.g = param['quadcopter_model']['g']
            self.m = param['quadcopter_model']['m']
            self.k = param['quadcopter_model']['k']
            self.A = param['quadcopter_model']['A']
            self.I = param['quadcopter_model']['I']
            self.l = param['quadcopter_model']['l']
            self.b = param['quadcopter_model']['b']
        else:
            errmsg = ('ERROR. Could not find parameters in %s...\n, shutting down node'%(str(self)))
            raise Exception(errmsg)
        self.param = param
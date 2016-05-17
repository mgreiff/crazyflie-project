# -*- coding: utf-8 -*-
import os,sys,inspect
from time import sleep

# Sets up paths and imports crazylib
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
modulesdir = os.path.dirname(curdir)
configdir = os.path.join(os.path.dirname(modulesdir),'config')
sys.path.insert(0,modulesdir)

from crazylib import print_progress


items = ["a", "b", "c", "d", "e"]

for ii in range(len(items)+1):
    sleep(1)
    print_progress(ii, len(items), prefix = 'Progress:', suffix = 'Complete', barLength = 50)

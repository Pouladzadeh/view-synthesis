#################################################################
# File: profile.py
#
# Created: 31-12-2016 by Parvaneh Pouladzadeh
# Last Modified: Mon Jan  2 15:26:24 2017
#
# Description:
#
#
#
# Copyright (C) 2016. All rights reserved.
#
#################################################################

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import subprocess as sp
import sys
import os.path
import glob
import csv

data_dir    = os.getcwd() + '/data'
output_dir  = os.getcwd() + '/output'
executable  = os.getcwd() + '/build/viewSynth'

do_eval = True
dump_app_errors = True

class Method(object):
    def __init__(self, name, version, num_threads, display):
        self.name = name
        self.version = version
        self.num_threads = num_threads
        self.display = display

datasets = [ 'balloons',
            'dog',
            ]

methods = [ Method('sequential', 'PTHREAD_NONE',   '1',     'true'),
            Method('scenario1-2', 'PTHREAD_V1',   '2',     'true'),
            Method('scenario1-4', 'PTHREAD_V1',   '4',     'true'),
            Method('scenario2-2', 'PTHREAD_V2',   '2',     'true'),
            Method('scenario2-4', 'PTHREAD_V2',   '4',      'true'),
          ]

def evaluate_method(method, setname, params, cnt):
    # create output dir if not exists
    output = os.path.join(output_dir, setname, method.name)
    if not os.path.exists(output):
        os.makedirs(output)

    proc = sp.Popen(['make', 'clean'], stdout=sp.PIPE, stderr=sp.PIPE, bufsize=1)
    proc.wait()
    make = ['make', 'VER=' + method.version]
    make += ['THREAD_N=' + method.num_threads]
    make += ['DISPLAY=' + method.display]
    proc = sp.Popen(make, stdout=sp.PIPE, stderr=sp.PIPE, bufsize=1)
    proc.wait()
    command =  [executable]
    command += [os.path.join(setname, params)]
    filename =  os.path.join(output, method.name+ '_time.txt')
    print(command)
    with open(filename,"wb") as out:
        proc = sp.Popen(command, stdout=out)
    proc.wait()

    valgrind_command = ['valgrind', '--tool=callgrind', '--separate-threads=yes']
    valgrind_command += ['--callgrind-out-file=', os.path.join(output, method.name + '_call.out')]
    valgrind_command += command
    print(valgrind_command)

    proc = sp.Popen(valgrind_command)
    return proc.wait()



if __name__=="__main__":

    evaluate_method(methods[1], datasets[0], 'viewsynthesis.cfg', 1)

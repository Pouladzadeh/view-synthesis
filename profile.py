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

data_dir    = '/home/ubuntu/VS_data/dataset_20/'
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

datasets = ['balloons_20',
            'kendo_20',
	    'champ_20',
	    'pant_20',
            ]

methods = [ Method('sequential', 'PTHREAD_NONE',   '1',     'false'),
            Method('scenario1-2', 'PTHREAD_V1',   '2',     'false'),
            Method('scenario1-4', 'PTHREAD_V1',   '4',     'false'),
            Method('scenario2-2', 'PTHREAD_V2',   '2',     'false'),
            Method('scenario2-4', 'PTHREAD_V2',   '4',      'false'),
          ]

def evaluate_method(method, setname, params, cnt):
    # create output dir if not exists
    output = os.path.join(output_dir, setname, method.name)
    if not os.path.exists(output):
        os.makedirs(output)
    print(os.path.join(output, method.name + '_call.out'))
    proc = sp.Popen(['make', 'clean'], stdout=sp.PIPE, stderr=sp.PIPE, bufsize=1)
    proc.wait()
    make = ['make', 'VER=' + method.version]
    make += ['THREAD_N=' + method.num_threads]
    make += ['DISPLAY=' + method.display]
    proc = sp.Popen(make, stdout=sp.PIPE, stderr=sp.PIPE, bufsize=1)
    proc.wait()
    command =  [executable]
    command += [os.path.join(data_dir, setname, params + '.cfg')]
    filename =  os.path.join(output, method.name+ '_time.txt')
    print(command)
    with open(filename,"wb") as out:
        proc = sp.Popen(command, stdout=out)
    proc.wait()

    command =  [executable]
    command += [os.path.join(data_dir, setname, params + '_call.cfg')]
    valgrind_command = ['valgrind', '--tool=callgrind', '--separate-threads=yes']
    valgrind_command += ['--callgrind-out-file=' + os.path.join(output, method.name + '_call.out')]
    valgrind_command += command
    print(valgrind_command)

    proc = sp.Popen(valgrind_command)
    return proc.wait()



if __name__=="__main__":

    for setname in datasets:
        for method in methods:
            print(setname)
            print(method.name)
            evaluate_method(method, setname, 'viewsynthesis', 1)
    print("Everything is done")

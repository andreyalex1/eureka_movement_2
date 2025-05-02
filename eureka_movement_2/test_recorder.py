#!/usr/bin/env python3.10

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

import os
import datetime

fname = str(datetime.datetime.now())
fname = fname + '.csv'
fname = fname.replace(" ", "_")


os.system ('ros2 topic echo /wheel_states --csv > /csv/' + fname)
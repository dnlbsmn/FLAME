#!/usr/bin/env python3

# To enable communication on the t7 shield run:
#sudo usermod -a -G dialout $USER 
#sudo chmod a+rw /dev/ttyUSB0

### ================================ ###
# INITIALISATION

# Importing all the ROS libraries
import rospy

# Importing all the flame libraries
from flame import ur_five
from flame import realsense
from flame import crystal_ball
from flame import motion_estimation

import time

### ================================ ###
# SYSTEM FUNCTIONS

# Starting up and connecting to all the devices
def start_devices():
    global carl, jerry, oracle
    carl.turn_on()
    jerry.turn_on()
    oracle.turn_on()

# The function that initialises the shutdown
def close_devices():
    global carl, jerry, oracle
    carl.turn_off()
    jerry.turn_off()
    oracle.turn_off()

### ================================ ###
# THE MAIN WORKFLOW OF THE SYSTEM

SIM = 1

if not SIM:
    if __name__ == '__main__':
        carl = realsense()
        jerry = ur_five()
        oracle = crystal_ball()

        # estimation_method = 'kalman filter 2'
        # oracle = motion_estimation(estimation_method)
        
        rospy.init_node('listener', anonymous = True)
        rospy.on_shutdown(close_devices)
        start_devices()

        rospy.spin()
    
else:
    print("Running simulation")
    #oracle = motion_estimation.particle_filter_fewer_states()
    estimation_method = 'particle filter 2'
    oracle = motion_estimation(estimation_method)
    f = open("../Benchmark_data/linear_12_10_train.csv", 'r')
    line = f.readline()
    while(1):
        line = f.readline()
        args = line.split(',')
        measurement = args[1:4]
        out = oracle.kf.run_filter(measurement, SIM)
        if out[0] == -999:
            break
        time.sleep(0.0001)
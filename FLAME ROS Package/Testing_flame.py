#!/usr/bin/env python3

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
    print("test")
    if __name__ == '__main__':
        carl = realsense()
        jerry = ur_five()
        # wong = crystal_ball()
        oracle = motion_estimation()
        
        rospy.init_node('listener', anonymous = True)
        rospy.on_shutdown(close_devices)
        start_devices()

        rospy.spin()
    
else:
    print("Running simulation")
    oracle = motion_estimation.particle_filter_fewer_states()
    f = open("../Benchmark_data/circular_50_05_train.csv", 'r')
    line = f.readline()
    while(1):
        line = f.readline()
        args = line.split(',')
        measurement = args[1:4]
        # measurement = [0.3, 0.3, 0.2]
        out = oracle.run_filter(measurement)
        # print(out)
        time.sleep(0.0001)
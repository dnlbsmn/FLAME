#!/usr/bin/env python3

### ================================ ###
# INITIALISATION

# Importing all the ROS libraries
import rospy

# Importing all the flame libraries
from flame import ur_five
from flame import realsense
from flame import crystal_ball

### ================================ ###
# SYSTEM FUNCTIONS

# Starting up and connecting to all the devices
def start_devices():
    global carl, jerry, wong
    carl.turn_on()
    jerry.turn_on()
    wong.turn_on()

# The function that initialises the shutdown
def close_devices():
    global carl, jerry, wong
    carl.turn_off()
    jerry.turn_off()
    wong.turn_off()

### ================================ ###
# THE MAIN WORKFLOW OF THE SYSTEM

if __name__ == '__main__':
    carl = realsense()
    jerry = ur_five()
    wong = crystal_ball()
    
    rospy.init_node('listener', anonymous = True)
    rospy.on_shutdown(close_devices)

    start_devices()

    rospy.spin()
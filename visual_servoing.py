#!/usr/bin/env python3

### ================================ ###
# INITIALISATION

# Importing all the basic Python libraries
import time
import threading

# Importing all the ROS libraries
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# Importing all the flame libraries
from flame import ur_five
from flame import realsense

### ================================ ###
# SYSTEM FUNCTIONS

# Starting up and connecting to all the devices
def start_devices(camera: realsense, arm: ur_five):
    rospy.init_node('listener', anonymous=True)

    camera.load_parameters()

    arm.connect()
    arm.begin_stream()

    # Beginning all the image subscribers
    depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, camera.process_depth)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_receive, [camera, arm], queue_size = 1)
    target_sub = rospy.Subscriber("/flame/predicted_position", Point, arm.send_target)

# A function that will execute every time a frame is received
def image_receive(image, system):
    # Getting the time that the image was received and calculate the time that matches the image capture
    image_time = time.time() - 0.034

    # Renaming the system object to make it more readable
    camera = system[0]
    arm = system[1]

    # Using the camera feed the the image position of the fruit are found
    kiwifruits = camera.process_image(image)

    # So long as a kiwifruit has been detected convert to global and send to the motion prediction algorithm
    if kiwifruits != []:
        kiwifruit = arm.locate_nearest(kiwifruits, image_time)
        arm.publish_target(kiwifruit, image_time)

# The thread to record the TCP poses constantly
def record_tcp(arm: ur_five):
    global running

    while running:
        arm.receive_tcp_pose()
        time.sleep(0.01)

# The function that initialises the shutdown
def end_program():
    global running
    running = 0

### ================================ ###
# THE MAIN WORKFLOW OF THE SYSTEM

if __name__ == '__main__':
    running = 1

    carl = realsense()
    jerry = ur_five()
    
    start_devices(carl, jerry)

    tcp_recorder = threading.Thread(target = record_tcp, args = (jerry, ))
    tcp_recorder.start()

    rospy.on_shutdown(end_program)

    rospy.spin()
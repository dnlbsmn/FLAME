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
from flame import nerian

### ================================ ###
# SYSTEM FUNCTIONS

# Starting up and connecting to all the devices
def start_devices(local_camera, global_camera, arm):
    rospy.init_node('listener', anonymous=True)

    local_camera.load_parameters()

    arm.connect()
    arm.begin_stream()

    init_point = Point()

    arm.send_variables(0)

    init_point.x = 0
    init_point.y = 0
    init_point.z = 0

    arm.send_pose(init_point)

    # Beginning all the image subscribers
    depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, local_camera.process_depth)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_receive, [local_camera, global_camera, arm], queue_size = 1)
    target_sub = rospy.Subscriber("/flame/predicted_position", Point, arm.send_pose)

# A function that will execute every time a frame is received
def image_receive(image, system):
    # Getting the time that the image was received and calculate the time that matches the image capture
    image_time = time.time() - 0.034

    # Renaming the system object to make it more readable
    local_camera = system[0]
    global_camera = system[1]
    arm = system[2]

    # Depending on the current state of the robot arm do different things
    ### 0 - LOCATE FRUIT - No servoing, gripper opened, waits for instruction
    if arm.pick_state == 0:
        # Awaiting the Nerian to send the first commands
        nerian_frame = rospy.wait_for_message('/nerian_stereo/left_image', Image, 10)
        kiwifruits = global_camera.process_image(nerian_frame)

        print(kiwifruits)

        # If no fruit are detected go into the finishing state otherwise send target point
        if kiwifruits != []:
            # Converting the image offset to a nerian position
            kiwifruit = arm.nerian_to_base(kiwifruits[0])

            # Send the target with a predetermined z coordinate
            target = Point()

            target.x = kiwifruit[0]
            target.y = kiwifruit[1]
            target.z = 0.4

            arm.send_pose(target)
        else:
            arm.pick_state = 5

    ### 1 - MOVE TO FRUIT - Servong to approximate position, gripper opened
    elif arm.pick_state == 1:
        # Nothing needs to be sent as the target position is already known
        pass

    ### 2 - VISUAL SERVOING - Servoing towards visible fruit
    elif arm.pick_state == 2:
        # Using the camera feed the image position of the fruit are found
        kiwifruits = local_camera.process_image(image)

        # So long as a kiwifruit has been detected convert to global and send to the motion prediction algorithm
        if kiwifruits != []:
            kiwifruit = arm.extract_target(kiwifruits, image_time)
            arm.publish_position(kiwifruit, image_time)

    ### 3 - GRABBING STATE - Continue servoing toward the fruit while closing the gripper
    elif arm.pick_state == 3:
        # Using the camera feed the image position of the fruit are found
        kiwifruits = local_camera.process_image(image)

        # So long as a kiwifruit has been detected convert to global and send to the motion prediction algorithm
        if kiwifruits != []:
            kiwifruit = arm.extract_target(kiwifruits, image_time)
            arm.publish_position(kiwifruit, image_time)

    ### 4 - DROP OFF SEQUENCE - Closes the gripper and drops off the fruit once it is closed
    elif arm.pick_state == 4:
        # Nothing needs to be sent when the arm is dropping off the kiwifruit
        pass

    ### 5 - RETURN TO START - Goes back to the starting position of the program
    elif arm.pick_state == 5:
        pass

    # Updating the state unless the end state has been called
    if arm.pick_state != 5:
        arm.update_variables()

    # Send the next state to the UR5
    arm.send_variables(arm.pick_state)

# Start the thread to record the TCP poses
def record_tcp(arm):
    # Update the UR5 TCP pose constantly until canceled or completed
    while (arm.pick_state != 5) & running:
        arm.receive_tcp_pose()
        time.sleep(0.01)

# Cleanly exiting the program
def stop(): 
    global running
    running = 0

### ================================ ###
# THE MAIN WORKFLOW OF THE SYSTEM

if __name__ == '__main__':
    running = 1

    carl = realsense()
    greg = nerian()
    jerry = ur_five()
    
    start_devices(carl, greg, jerry)

    tcp_recorder = threading.Thread(target = record_tcp, args = (jerry, ))
    tcp_recorder.start()

    rospy.on_shutdown(stop)

    # Waiting until the end state has been called
    while jerry.pick_state != 5:
        time.sleep(1)    
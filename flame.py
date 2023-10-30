#!/usr/bin/env python3

### ================================ ###
# INITIALISATION

# Importing all the basic Python libraries
import time
import math
import cv2 as cv
import numpy as np

# Importing all the ROS libraries
import rospy
from cv_bridge import CvBridge

# Importing the ros message types
from sensor_msgs.msg import CameraInfo
from nerian_stereo.msg import StereoCameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image

# Import all the other bits
import rtdeState

### ================================ ###
# DEFINING THE SYSTEM OBJECTS

# Defining the ur five input and output communication
class ur_five:
    ### OBJECT INITIALISATION ###
    # Starting up the global variables
    def __init__(self, fruit = True):
        # Declaring the rate at which the robot can run
        self.target_time = time.time()
        self.delay_time = 0.0

        # Creating a shift register to store a large amount of previous TCPs
        self.tcps = [0] * 100

        # Creating variables to store the active poses of the objects
        self.actual_tcp = [0, 0, 0, 0, 0, 0]
        self.target_tcp = [0, 0, 0, 0, 0, 0]
        self.realsense = [0, 0, 0, 0, 0, 0]

        # The offset of the RGB realsense camera and the TCP
        self.realsense_offset = [0.215, 0.0, 0.055]

        # Defining the desired distance from the TCP for fruit and flowers
        if fruit:
            self.tcp_offset = 0.230
        else:
            self.tcp_offset = 0.260

        # Defining a variable to track the state according to the UR5 or the computer
        self.pick_state = 0

    # Connect to the UR5 robot
    def connect(self):
        ROBOT_HOST = '192.168.10.20'
        config_filename = 'rtdeCommand.xml'

        self.rtde = rtdeState.RtdeState(ROBOT_HOST, config_filename)

        self.rtde.initialize()

    # Begin streaming to the UR5
    def begin_stream(self):
        # Start the stream to output for testing
        self.position_pub = rospy.Publisher("flame/perceived_position", Float64MultiArray, queue_size = 1)
        self.error_pub = rospy.Publisher("flame/position_error", Point, queue_size = 1)

    ### COMMUNICATION FUNCTIONS ###
    # Given a certain time stamp it will set the current tcp pose to that of the nearest time match
    def set_tcp_pose(self, pose_time):
        # Set a flag to indicate that a tcp match has been made
        too_early = 1

        # Iterate through all the tcp readings until one is later than the desired time
        for tcp in self.tcps:
            if tcp != 0:
                if tcp[1] < pose_time:
                    self.actual_tcp = tcp[0]
                    too_early = 0
                    break

        # print("image time: " + str(pose_time))
        # print("  tcp time: " + str(tcp[1]))

        # If no matching tcp pose was found in memory
        if too_early:
            print("TCP memory does not go that far back")
            return

        # Update the position of the realsense to match the tcp
        self.update_realsense_pose()

    # Appends the current pose of the UR5 to the array of poses
    def receive_tcp_pose(self):
        # Update the state of the robot
        state = self.rtde.receive()
        time_stamp = time.time()

        tcp = [0, 0, 0, 0, 0, 0]

        # Read the current position of the UR5
        tcp[0] = state.actual_TCP_pose[0]
        tcp[1] = state.actual_TCP_pose[1]
        tcp[2] = state.actual_TCP_pose[2]
        tcp[3] = state.actual_TCP_pose[3]
        tcp[4] = state.actual_TCP_pose[4]
        tcp[5] = state.actual_TCP_pose[5]

        # Shift all the values across the register
        self.tcps[1:] = self.tcps[:-1]

        # Append the most recent reading to the start of the list
        self.tcps[0] = [tcp, time_stamp]

    # Refreshes the realsense pose according to the current TCP pose
    def update_realsense_pose(self):
        rotated = self.rotate_point(self.realsense_offset[0:2], self.actual_tcp[5])

        self.realsense[0] = self.actual_tcp[0] + rotated[0]
        self.realsense[1] = self.actual_tcp[1] + rotated[1]
        self.realsense[2] = self.actual_tcp[2] + self.realsense_offset[2]

        # print("RSC X: " + str(round(self.realsense[0] * 1000)), end=" mm ")
        # print("Y: " + str(round(self.realsense[1] * 1000)), end=" mm ")
        # print("Z: " + str(round(self.realsense[2] * 1000)), end=" mm\n")

    # Updates all the variables of the code to match that of the actual UR5
    def update_state(self):
        state = self.rtde.receive()

        # Read the current position of the UR5
        self.actual_tcp[0] = state.actual_TCP_pose[0]
        self.actual_tcp[1] = state.actual_TCP_pose[1]
        self.actual_tcp[2] = state.actual_TCP_pose[2]
        self.actual_tcp[3] = state.actual_TCP_pose[3]
        self.actual_tcp[4] = state.actual_TCP_pose[4]
        self.actual_tcp[5] = state.actual_TCP_pose[5]

        # Read the next state variable
        self.pick_state = state.output_int_register_0

    ### CALLBACK FUNCTIONS ###
    # Publishes the target to the predictive motion node
    def publish_target(self, position, position_time):
        # If the position is empty dont try to send it
        if position == []:
            return

        # Publish the global position of the detected fruit for data recording
        pub_point = Float64MultiArray()

        point_data = [0, 0, 0, 0]
        point_data[0] = position[0]
        point_data[1] = position[1]
        point_data[2] = position[2]
        point_data[3] = position_time

        pub_point.data = point_data
        self.position_pub.publish(pub_point)

        # Calculates the error of the current target and publishes it to the ros topic
        error = Point()

        error.x = position[0] - self.actual_tcp[0]
        error.y = position[1] - self.actual_tcp[1]
        error.z = position[2] - self.actual_tcp[2]

        self.error_pub.publish(error)

    ### ACTION FUNCTIONS ###
    # Given a point relative to the Realsense convert and send a target to the UR5
    def send_target(self, position: Point):
        target = [0, 0, 0, 0, 0, 0]

        target[0] = position.x
        target[1] = position.y
        target[2] = position.z

        # If any of the movement limits are violated do not send the target
        if (position.x < 0.2) or (position.x > 0.84):
            print("Position out of range x")
            return
        if (position.y < -0.6) or (position.y > 0.6):
            print("Position out of range y")
            return
        if position.z > 0.64:
            print("Position out of range z")
            return

        if time.time() >= self.target_time:
            # Upload the target to the UR5 robotic arm
            for i in range(0, len(target)):
                self.rtde.target_point.__dict__["input_double_register_%i" % i] = target[i]
            self.rtde.con.send(self.rtde.target_point)

            # Set the next time which a message should be sent
            self.target_time = time.time() + self.delay_time

            # print("X: " + str(round(target[0] * 1000)), end=" mm ")
            # print("Y: " + str(round(target[1] * 1000)), end=" mm ")
            # print("Z: " + str(round(target[2] * 1000)), end=" mm\n")

    # Either enable or disable the servoing variable on the robot
    def send_state(self, servoing = 0):
        # The following are the names of the states and their purpose
        # 0 - LOCATE FRUIT - No servoing, gripper opened, waits for instruction
        # 1 - MOVE TO FRUIT - Servong to approximate position, gripper opened
        # 2 - VISUAL SERVOING - Servoing towards visible fruit
        # 3 - GRABBING STATE - Continue servoing toward the fruit while closing the gripper
        # 4 - DROP OFF SEQUENCE - Closes the gripper and drops off the fruit once it is closed
        # 5 - RETURN TO START - Goes back to the starting position of the program

        # Signal to the UR5 that it may execute a servoing command
        self.rtde.servo.input_int_register_0 = servoing
        self.rtde.con.send(self.rtde.servo)

    ### TARGET MANIPULATION FUNCTIONS ###
    # Given a list of candidate targets relative to the realsense determines the position of the most likely target
    def extract_target(self, targets, image_time):
        # Updates the pose values of the UR5 and realsense
        self.set_tcp_pose(image_time)

        min_error = 1
        best_position = []
        
        # Convert all the targets to global coordinates and evaluate the best one
        for target in targets:
            position = self.realsense_to_position([target.x, target.y, target.z])
            error = abs(self.actual_tcp[0] - position[0]) + abs(self.actual_tcp[1] - position[1]) + abs(self.actual_tcp[2] - position[2])

            if error < min_error:
                best_position = position
                min_error = error

        return best_position

    # Rotates the position of a point (x, y) about the origin by a given angle
    def rotate_point(self, point, angle):
        x = point[0]
        y = point[1]

        # Converting the point to polar coordinates
        radius = math.sqrt(x * x + y * y)
        theta = math.atan2(y, x)

        # Rotating the polar coordinates
        theta_rot = theta + angle

        # Converting the polar coordinates back
        x_rot = radius * math.cos(theta_rot)    
        y_rot = radius * math.sin(theta_rot)    

        return [x_rot, y_rot]

    # Takes the position relative to the realsense and converts it to global coordinates
    def realsense_to_position(self, point):
        # Rotating the XZ plane clockwise by theta degrees
        theta = 31.49 * math.pi / 180

        x_rot = - point[0] * math.cos(theta) - point[2] * math.sin(theta)
        z_rot = point[0] * math.sin(theta) - point[2] * math.cos(theta)

        # Translating the position to align the centre of the TCP to the camera
        x_trn = x_rot + self.realsense_offset[0]
        y_trn = - point[1]
        z_trn = - z_rot + self.realsense_offset[2]

        # Rotating the x and y coordinates of the point according to the rotation of the TCP
        xy_rotated = self.rotate_point([y_trn, x_trn], self.actual_tcp[5])

        # Applying the transform noting that the axis of the realsense are rotated by 90
        x = self.actual_tcp[0] + xy_rotated[1]
        y = self.actual_tcp[1] + xy_rotated[0]
        z = self.actual_tcp[2] + z_trn - self.tcp_offset

        # print(" X: " + str(round(xy_rotated[0] * 1000)), end="")
        # print(" Y: " + str(round(xy_rotated[1] * 1000)), end="")
        # print(" Z: " + str(round(z_trn * 1000)))
        
        return [x, y, z]

    # Takes the position relative to the nerian and converts it to global coordinates
    def nerian_to_position(self, point: Point):
        x = - point.y + 0.6
        y = point.x - 0.025
        z = point.z - 0.65

        return [x, y, z]

# Defining the camera in hand input and image processing object
class realsense:
    ### OBJECT INITIALISATION ###
    # Initialise the default values of the HSV filter
    def __init__(self, fruit = True):
        # Initialising the image processing values
        self.bridge = CvBridge()

        # Changing the way that the realsense perceives position depending on target type
        if fruit:
            self.target_depth = 25
            self.hsv_ranges = [(15, 80, 0), (25, 255, 255)]
        else:
            self.target_depth = 0
            self.hsv_ranges = [(20, 80, 40), (35, 255, 255)]

    # Loading all the camera parameters for future calculations
    def load_parameters(self):
        # Waiting for camera entrinsics to be sent
        K_matrix = rospy.wait_for_message('/camera/aligned_depth_to_color/camera_info', CameraInfo).K

        # Storing the entrinsics for future calculations
        self.x_focal = K_matrix[0]
        self.y_focal = K_matrix[4]
        self.x_centre = K_matrix[2]
        self.y_centre = K_matrix[5]

    # Load in the YOLO model
    def load_yolo(self):
        # Importing the machine learning stuff
        from ultralytics import YOLO

        self.fruit_detector = YOLO("flame_fake_kiwifruit.pt")

    ### CALLBACK FUNCTIONS ###
    # Save the most recent depth image into a variable
    def process_depth(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data)

    # Processing an RGB image when received
    def process_image(self, data):
        # Converting the received image and extracting the fruit from it
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        fruits = self.extract_fruit(image)

        # Converting the image position of the fruit to a spacial position
        fruit_locations = []

        if fruits is not None:
            for fruit in fruits:
                fruit_locations.append(self.image_to_position(fruit))
                cv.circle(image, (fruit[0], fruit[1]), 3, (255, 0, 255), -1)

        cv.imshow("window", image)
        cv.waitKey(1)

        return fruit_locations

    ### VISION AND LOCATING FUNCTIONS ###
    # Extracting a fruit image positions using thresholding
    def extract_fruit(self, frame):
        # Converting the image to HSV and extracting kiwifruit
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_mask = cv.inRange(frame_hsv, self.hsv_ranges[0], self.hsv_ranges[1])

        cv.imshow("one", frame_mask)
        cv.waitKey(1)

        # Applying some morphological filters
        kernel_open = cv.getStructuringElement(cv.MORPH_ELLIPSE, [5, 5])
        kernel_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, [23, 23])

        frame_mask = cv.morphologyEx(frame_mask, cv.MORPH_OPEN, kernel_open)

        # cv.imshow("two", frame_mask)
        # cv.waitKey(1)

        frame_mask = cv.morphologyEx(frame_mask, cv.MORPH_CLOSE, kernel_close)

        # cv.imshow("three", frame_mask)
        # cv.waitKey(1)

        # Finding the contours of the potential fruit
        contours, heirarchy = cv.findContours(frame_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        frame_mask = cv.drawContours(frame_mask, contours, -1, 127, 3)

        # cv.imshow("four", frame_mask)
        # cv.waitKey(1)

        fruits = []

        # Test if the contours are large enough
        for contour in contours:
            if cv.contourArea(contour) > 400:
                # Do position calculations if it is
                m = cv.moments(contour)
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])

                fruits.append([cx, cy])

        return fruits

    # Extract the fruit from the target image except using YOLO
    def extract_fruit_yolo(self, frame):
        results = self.fruit_detector.predict(frame, save=False, show=False, conf=0.2, imgsz = (448, 256), boxes = False)

        fruits = []

        for fruit in results[0].boxes:
            cx = int((fruit.xyxy[0][2] + fruit.xyxy[0][0]) / 2)
            cy = int((fruit.xyxy[0][3] + fruit.xyxy[0][1]) / 2)

        fruits.append([cx, cy])

        return fruits

    # Converting an image position to a cartesian position
    def image_to_position(self, point):
        # Returning the error for the control system to deal with
        position = Point()

        # Doing the calculations
        position.z = 0.001 * self.get_depth(point)
        position.x = 0.001 * float((point[0] - self.x_centre) * position.z * 1000) / self.x_focal
        position.y = 0.001 * float((point[1] - self.y_centre) * position.z * 1000) / self.y_focal

        # Printing out the errors
        # print("X Error: " + str(round(position.x * 1000)) + " mm ", end="")
        # print("Y Error: " + str(round(position.y * 1000)) + " mm ", end="")
        # print("Z Error: " + str(position.z * 1000) + " mm ")

        return position
    
    # Given a point on the image the average depth of the neighborhood will be found and offset applied
    def get_depth(self, point):
        # Defining the boundaries of the area of extraction
        sample_radius = 5

        x_min = point[1] - sample_radius
        x_max = point[1] + sample_radius
        y_min = point[0] - sample_radius
        y_max = point[0] + sample_radius

        # Extracting and copying a data segment from the current depth image
        depth_segment = np.copy(self.depth_image[x_min:x_max, y_min:y_max])

        # Finding the number of valid readings in the depth segment
        num_readings = sum(sum(sum([depth_segment != 0])))
        
        # Getting the average reading of the depth segment excluding zeros
        depth_reading = sum(sum(depth_segment)) / num_readings

        # Adding on 25 mm to get the approximate centre of the kiwifruit
        depth_reading += self.target_depth

        return depth_reading
    
# Defininf the global camera input and output functions
class nerian:
    # Initialising all the variables
    def __init__(self):
        # Initialising the image processing values
        self.bridge = CvBridge()

        self.hsv_ranges = [(15, 100, 40), (25, 255, 255)]

    # Getting all the parameters of the camera
    def load_parameters(self):
        # Waiting for camera entrinsics to be sent
        camera_info = rospy.wait_for_message('/nerian_stereo/stereo_camera_info', StereoCameraInfo)

        K_matrix = camera_info.left_info.K
        self.Q = camera_info.Q

        # Storing the entrinsics for future calculations
        self.x_focal = K_matrix[0]
        self.y_focal = K_matrix[4]
        self.x_centre = K_matrix[2]
        self.y_centre = K_matrix[5]

    # Processing an image of the nerian
    def process_image(self, data):
        # Retrieving the most nearest disparity map
        disparity_data = rospy.wait_for_message('/nerian_stereo/disparity_map', Image)
        disparity_map = self.bridge.imgmsg_to_cv2(disparity_data)

        # Converting the received image and extracting the fruit from it
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        fruits = self.extract_fruit(image)

        # Converting the image position of the fruit to a spacial position
        fruit_locations = []

        if fruits is not None:
            for fruit in fruits:
                fruit_locations.append(self.image_to_position(fruit[0], fruit[1]))
                cv.circle(image, (fruit[0], fruit[1]), 7, (255, 0, 255), -1)

        cv.namedWindow("nerian", cv.WINDOW_NORMAL)
        cv.resizeWindow("nerian", 800, 600)
        cv.imshow("nerian", image)
        cv.waitKey(1)

        return fruit_locations

    # Extracting the fruit out of the nerian frame
    def extract_fruit(self, frame):
        # Converting the image to HSV and extracting kiwifruit
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_mask = cv.inRange(frame_hsv, self.hsv_ranges[0], self.hsv_ranges[1])

        # Applying some morphological filters
        kernel_open = cv.getStructuringElement(cv.MORPH_ELLIPSE, [29, 29])
        kernel_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, [5, 5])

        frame_mask = cv.morphologyEx(frame_mask, cv.MORPH_CLOSE, kernel_close)
        frame_mask = cv.morphologyEx(frame_mask, cv.MORPH_OPEN, kernel_open)

        # Finding the contours of the potential fruit
        contours, heirarchy = cv.findContours(frame_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Sorting the fruits from biggest to smallest
        contours = sorted(contours, key = cv.contourArea, reverse = True)

        fruits = []

        # Test if the contours are large enough
        for contour in contours:
            if cv.contourArea(contour) > 1500:
                # Do position calculations if it is
                m = cv.moments(contour)
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])

                fruits.append([cx, cy])

        return fruits
    
    # Given a single image position calculates the position assuming the object is at a given depth
    def image_to_position(self, image_x: int, image_y: int) -> Point:
        # Pixel to millimeter conversion experiments
        # [476, 477] -> [1401, 535] = 600 mm

        pixel_to_millimeter = 0.645

        # Calculating depth in a simpler way
        position = Point()

        # Calculating depth using focal length and baseline and disparity
        position.z = 1.4
        position.x = 0.001 * (image_x - 968) * pixel_to_millimeter
        position.y = 0.001 * (image_y - 608) * pixel_to_millimeter

        return position

        # # Calculating the intermediary points of the position by doing the matrix multiplication: (x', y', z', w) = Q * (u, v, d, 1)'
        # x_dash = self.Q[0] * image_x + self.Q[3]
        # y_dash = self.Q[5] * image_y + self.Q[7]
        # z_dash = self.Q[11]
        # w = self.Q[14] * disparity

        # # Taking these values and converting them to actual coordinates: (x, y, z) = 1 / w * (x', y', z')
        # position = Point()

        # position.x = x_dash / w
        # position.y = y_dash / w
        # position.z = z_dash / w

        # return position
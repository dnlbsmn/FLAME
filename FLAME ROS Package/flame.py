#!/usr/bin/env python3

### ================================ ###
# INITIALISATION

# Importing all the basic Python libraries
import time
import math
import cv2 as cv
import numpy as np
import threading
import matplotlib.pyplot as plt

# Importing all the ROS libraries
import rospy
from cv_bridge import CvBridge

# Importing the ros message types
from std_msgs.msg import Header

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from nerian_stereo.msg import StereoCameraInfo

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

# Import filterpy
from filterpy.kalman import KalmanFilter
from filterpy.kalman import ExtendedKalmanFilter
import scipy
from filterpy.common import Q_discrete_white_noise
from random import random

# Import all the other bits
import rtdeState

### ================================ ###
# DEFINING THE SYSTEM OBJECTS

# Defining the ur five input and output communication
class ur_five:
    ### BASIC INTERFACING ###
    # Turn on the ur_five object
    def turn_on(self):
        self.on = True
        
        # Connecting to the realtime data exchange
        self.rtde = rtdeState.RtdeState('192.168.10.20', 'rtdeCommand.xml')
        self.rtde.initialize()

        # Start the stream that publishes the position of the target
        self.position_pub = rospy.Publisher("flame/perceived_position", PointStamped, queue_size = 1)
        self.error_pub = rospy.Publisher("flame/position_error", Point, queue_size = 1)

        # Starting the workflow of predicted position to RTDE
        self.target_sub = rospy.Subscriber("/flame/predicted_position", Point, self.upload_pose)
        self.fruits_sub = rospy.Subscriber("/flame/realsense_positions", PoseArray, self.servo_realsense)

        # Start the thread to record the TCP poses of the UR5
        tcp_recorder = threading.Thread(target = self.record_tcp)
        tcp_recorder.start()

    # Turn off the ur_five object
    def turn_off(self):
        # Disconnecting all the cables
        self.position_pub.unregister()
        self.error_pub.unregister()
        self.target_sub.unregister()
        
        self.on = False

    ### INITIALISATION ###
    # Starting up the global variables
    def __init__(self, fruit = True):
        # Starting the object as off
        self.on = False

        # Declaring the rate at which the robot can run
        self.target_time = time.time()
        self.delay_time = 0.0

        # Creating a shift register to store a large amount of previous TCPs
        self.tcps = [0] * 100

        # Creating variables to store the active poses of the objects
        self.actual_tcp = [0, 0, 0, 0, 0, 0]

        # The offset of the RGB realsense camera and the TCP
        self.realsense_offset = [-0.215, 0.0, -0.055]

        # Defining the desired distance from the TCP for fruit and flowers
        if fruit:
            self.tcp_offset = [0, 0, 0.230]
        else:
            self.tcp_offset = [0, 0, 0.260]

        # Defining a variable to track the state according to the UR5 or the computer
        self.pick_state = 0

    # The function that continuously records the TCP pose of the UR5 when it is on
    def record_tcp(self) -> None:
        while self.on:
            self.download_pose()
            time.sleep(0.01)

    ### RTDE COMMUNICATION ###
    # Appends the current pose of the UR5 to the array of poses
    def download_pose(self) -> None:
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

    # Uploads a pose to the registers used for communicating pose with the RTDE
    def upload_pose(self, position: Point) -> None:
        target = [0, 0, 0, 0, 0, 0]

        # Dealing with the translation first
        target[0] = position.x
        target[1] = position.y
        target[2] = position.z

        # If any of the movement limits are violated do not upload the target
        if (position.x < 0.2) or (position.x > 0.84):
            print("Position out of range x")
            return
        if (position.y < -0.6) or (position.y > 0.6):
            print("Position out of range y")
            return
        if position.z > 0.64:
            print("Position out of range z")
            print(position.z)
            return
        
        # Dealing with the rotation next
        rotation = self.get_rotation()

        # If any of the movement limits are violated do not upload the target
        if (rotation.x < -0.6) or (rotation.x > 0.6):
            print("Rotation out of range x")
            return
        if (rotation.y < -0.6) or (rotation.y > 0.6):
            print("Rotation out of range y")
            return
        if (rotation.y < -0.6) or (rotation.y > 0.6):
            print("Rotation out of range z")
            return

        target[3] = rotation.x
        target[4] = rotation.y
        target[5] = rotation.z

        # If time greater or equal to the delay time has passed, upload the target position
        if time.time() >= self.target_time:
            # Upload the target to the UR5 robotic arm
            for i in range(0, len(target)):
                self.rtde.target_point.__dict__["input_double_register_%i" % i] = target[i]
            self.rtde.con.send(self.rtde.target_point)

            # Set the next time which a message should be sent
            self.target_time = time.time() + self.delay_time

    # Updates all the variables of the code to match that of the actual UR5
    def download_variables(self) -> None:
        state = self.rtde.receive()

        # Read the state variable from the UR5
        self.pick_state = state.output_int_register_0

    # Either enable or disable the servoing variable on the robot
    def upload_variables(self, state: int) -> None:
        # Signal to the UR5 that it may execute a servoing command
        self.rtde.servo.input_int_register_0 = state
        self.rtde.con.send(self.rtde.servo)

    ### ROSTOPIC COMMUNICATION ###
    # Publishes the target to the predictive motion node
    def publish_position(self, position: Point, position_time: float) -> None:
        # If the position is empty dont try to send it
        if position == None:
            return

        # Publish the global position of the detected fruit for data recording
        pub_point = PointStamped()

        pub_point.point.x = position.x
        pub_point.point.y = position.y
        pub_point.point.z = position.z

        pub_point.header.stamp.secs = int(position_time)
        pub_point.header.stamp.nsecs = int((position_time % 1) * 10 ** 9)

        self.position_pub.publish(pub_point)

        # Calculates the error of the current target and publishes it to the ros topic
        error = Point()

        error.x = position.x - self.actual_tcp[0]
        error.y = position.y - self.actual_tcp[1]
        error.z = position.z - self.actual_tcp[2]

        self.error_pub.publish(error)

    ### POSITION MANIPULATION ###
    # Converts a coordinate relative to the TCP to a coordinate relative to the base
    def tcp_to_base(self, point: Point) -> Point:
        # If no TCP pose has been sent yet, do not attempt to perform the calculation
        if self.actual_tcp == [0,0,0,0,0,0]:
            return

        # First rotating the position of the fruit relative to the global axes
        # Note that the angles of x and y are flipped to counteract some flipping action done earlier
        rx = - self.actual_tcp[3]
        ry = - self.actual_tcp[4]
        rz = self.actual_tcp[5]

        # Converting the pose to axis angle notation
        theta = math.sqrt(rx * rx + ry * ry + rz * rz)

        ux = rx / theta
        uy = ry / theta
        uz = rz / theta

        # Converting the axis angle to a rotation matrix per term for x, y and z 
        # Refer to the Wikipedia article on rotation matrix: rotation matrix from axis angle

        # Precalculating the sine and cosine for readability and efficiency
        cos = math.cos(theta)
        sin = math.sin(theta)

        # The notation here refers to the row coordinate then the column
        # For example xy refers to the proportion of old y in the new x
        xx = cos + ux * ux * (1 - cos)
        xy = uy * ux * (1 - cos) + uz * sin
        xz = uz * ux * (1 - cos) - uy * sin

        yx = ux * uy * (1 - cos) - uz * sin
        yy = cos + uy * uy * (1 - cos)
        yz = uz * uy * (1 - cos) + ux * sin

        zx = ux * uz * (1 - cos) + uy * sin
        zy = uy * uz * (1 - cos) - ux * sin
        zz = cos + uz * uz * (1 - cos)

        # Multiplying the per term rotation by the inputted point
        rotated_point = Point()

        rotated_point.x = point.x * xx + point.y * xy + point.z * xz
        rotated_point.y = point.x * yx + point.y * yy + point.z * yz
        rotated_point.z = point.x * zx + point.y * zy + point.z * zz

        # Secondly applying the offset of the TCP position
        transformed_point = Point()

        transformed_point.x = rotated_point.x + self.actual_tcp[0]
        transformed_point.y = rotated_point.y + self.actual_tcp[1]
        transformed_point.z = rotated_point.z + self.actual_tcp[2]

        # print("base x: " + str(round(transformed_point.x, 2)), end="")
        # print(" y: " + str(round(transformed_point.y, 2)), end="")
        # print(" z: " + str(round(transformed_point.z, 2)))

        return transformed_point

    # Converts a coordinate given relative to the realsense to a coordinate relative to the TCP
    def realsense_to_tcp(self, point: Point) -> Point:
        # Applying a rotation of the realsense relative to the TCP
        # Converting the rotation angle to radians
        theta = 31.5 * math.pi / 180

        # Flipping the direction of the x and y axes
        x_1 = - point.x
        y_1 = - point.y
        z_1 = point.z

        # Rotating the x and z axes
        x_2 = x_1 * math.cos(theta) - z_1 * math.sin(theta)
        y_2 = y_1
        z_2 = x_1 * math.sin(theta) + z_1 * math.cos(theta)

        # Applying a translation of the realsense relative to the UR5 TCP
        x_3 = x_2 - self.realsense_offset[0]
        y_3 = y_2 - self.realsense_offset[1]
        z_3 = z_2 - self.realsense_offset[2]

        # Applying a translation to account for the desired TCP offset
        transformed_point = Point()

        transformed_point.x = x_3 - self.tcp_offset[0]
        transformed_point.y = y_3 - self.tcp_offset[1]
        transformed_point.z = z_3 - self.tcp_offset[2]

        # print("tcp x: " + str(round(transformed_point.x, 2)), end="")
        # print(" y: " + str(round(transformed_point.y, 2)), end="")
        # print(" z: " + str(round(transformed_point.z, 2)))

        return transformed_point

    # Takes the position relative to the nerian and converts it to global coordinates
    def nerian_to_base(self, point: Point) -> Point:
        position = Point()
        
        position.x = - point.y + 0.6
        position.y = point.x - 0.025
        position.z = point.z - 0.65

        return position
    
    ### MISCELLANEOUS ###
    # Given a list of candidate targets relative to the realsense determines the position of the closest target and the global coordinates
    def servo_realsense(self, targets: PoseArray) -> None:
        # Declaring the time when the image was captured
        image_time = float(targets.header.stamp.secs) + float(targets.header.stamp.nsecs) / (10 ** 9)

        # Updates the pose values of the UR5 and realsense
        self.update_pose(image_time)

        min_error = 1
        
        # Convert all the targets to TCP coordinates and evaluate the best one according to distance from the TCP
        for target in targets.poses:
            # Comparing the distance from the TCP to the fruit
            position = self.realsense_to_tcp(target.position)
            error = abs(position.x) + abs(position.y) + abs(position.z)

            # Calculating if this is a minimum reading and saving it if it is
            if error < min_error:
                best = position
                min_error = error

        # Calculating the global position of the kiwifruit
        best_position = self.tcp_to_base(best)

        # Publish the closest position to the motion estimator
        self.publish_position(best_position, image_time)

    # Returns the rotation value of the UR5 as a point
    def get_rotation(self) -> Point:
        rotation = Point()

        # Approximate the rotation about the y axis to be a sine wave
        rotation.x = 0
        rotation.y = 0 # 0.5 * math.sin(0.5 * (2 * math.pi * time.time()))
        rotation.z = 0

        return rotation

    # Given a certain time stamp it will set the current tcp pose to that of the nearest time match
    def update_pose(self, pose_time) -> None:
        # Iterate through all the tcp readings until one is later than the desired time
        for tcp in self.tcps:
            if (tcp != 0):
                if (tcp[1] < pose_time):
                    self.actual_tcp = tcp[0]
                    break

# Defining the camera in hand input and image processing object
class realsense:
    ### BASIC INTERFACING ###
    # Connecting up all the internals and turning it on
    def turn_on(self):
        # Waiting for camera intrinsics to be sent
        K_matrix = rospy.wait_for_message('/camera/aligned_depth_to_color/camera_info', CameraInfo).K

        # Storing the entrinsics for future calculations
        self.x_focal = K_matrix[0]
        self.y_focal = K_matrix[4]
        self.x_centre = K_matrix[2]
        self.y_centre = K_matrix[5]

        # Starting up all the ROS subscribers of the image streams
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.process_depth, queue_size = 1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.process_image, queue_size = 1)

        # Starting up all the ROS publishers for outputing the kiwifruit point
        self.position_pub = rospy.Publisher("/flame/realsense_positions", PoseArray, queue_size = 1)

        self.on = True

    # Shutting it down elegantly 
    def turn_off(self):
        # Disconnecting all the cables
        self.depth_sub.unregister()
        self.image_sub.unregister()

        self.on = False

    ### INITIALISATION ###
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

        # The expected delay of capturing an image
        self.delay = 0.034

    # Load in the YOLO model
    def load_yolo(self):
        # Importing the machine learning stuff
        from ultralytics import YOLO

        self.fruit_detector = YOLO("flame_fake_kiwifruit.pt")

    ### IMAGE PROCESSING ###
    # Extracting a fruit image positions using thresholding
    def extract_fruit(self, frame):
        # Converting the image to HSV and extracting kiwifruit
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_mask = cv.inRange(frame_hsv, self.hsv_ranges[0], self.hsv_ranges[1])

        # cv.imshow("one", frame_mask)
        # cv.waitKey(1)

        # Applying some morphological filters
        kernel_open = cv.getStructuringElement(cv.MORPH_ELLIPSE, [5, 5])
        kernel_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, [23, 23])

        frame_mask = cv.morphologyEx(frame_mask, cv.MORPH_OPEN, kernel_open)
        frame_mask = cv.morphologyEx(frame_mask, cv.MORPH_CLOSE, kernel_close)

        # Finding the contours of the potential fruit
        contours, heirarchy = cv.findContours(frame_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        frame_mask = cv.drawContours(frame_mask, contours, -1, 127, 3)

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
    def image_to_realsense(self, image_position) -> Point:
        # Returning the error for the control system to deal with
        position = Point()

        # Doing the calculations to locate the 3D position of a given pixel on the image
        position.z = 0.001 * self.get_depth(image_position)
        position.x = 0.001 * float((image_position[0] - self.x_centre) * position.z * 1000) / self.x_focal
        position.y = 0.001 * float((image_position[1] - self.y_centre) * position.z * 1000) / self.y_focal

        return position
    
    # Given a point on the image the average depth of the neighborhood will be found and offset applied
    def get_depth(self, image_position) -> float:
        # Defining the boundaries of the area of extraction
        sample_radius = 5

        x_min = image_position[1] - sample_radius
        x_max = image_position[1] + sample_radius
        y_min = image_position[0] - sample_radius
        y_max = image_position[0] + sample_radius

        # Extracting and copying a data segment from the current depth image
        depth_segment = np.copy(self.depth_image[x_min:x_max, y_min:y_max])

        # Finding the number of valid readings in the depth segment
        num_readings = sum(sum(sum([depth_segment != 0])))
        
        # Getting the average reading of the depth segment excluding zeros
        depth_reading = sum(sum(depth_segment)) / num_readings

        # Adding on 25 mm to get the approximate centre of the kiwifruit
        depth_reading += self.target_depth

        return depth_reading
  
    ### CALLBACKS ###
    # Save the most recent depth image into a variable
    def process_depth(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data)

    # Processing an RGB image when received
    def process_image(self, data):
        # Recording the time prior to any preprocessing
        image_time = time.time() - self.delay

        # Formatting the time for the header object
        header = Header()
        header.stamp.secs = int(image_time)
        header.stamp.nsecs = int((image_time % 1) * 10 ** 9)

        # Converting the received image and extracting the fruit from it
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        fruits = self.extract_fruit(image)

        # Converting the image position of the fruit to a spacial position
        fruit_poses = []

        if fruits is not None:
            for fruit in fruits:
                # Displaying the fruit on the image
                cv.circle(image, (fruit[0], fruit[1]), 3, (255, 0, 255), -1)

                # Saving the fruit poses
                fruit_pose = Pose()
                fruit_pose.position = self.image_to_realsense(fruit)
                fruit_poses.append(fruit_pose)

        cv.imshow("window", image)
        cv.waitKey(1)

        fruit = PoseArray()
        fruit.header = header
        fruit.poses = fruit_poses

        # Publishing all the fruit poses that were identified
        self.position_pub.publish(fruit)
  
# Defining the global camera input and output functions
class nerian:
    ### INITIALISATION ###
    # Initialising all the variables
    def __init__(self, fruit):
        self.on = False

        # Initialising the image processing values
        self.bridge = CvBridge()

        if fruit:
            self.hsv_ranges = [(15, 100, 40), (25, 255, 255)]

    # Turning on and getting all the parameters of the camera
    def turn_on(self):
        # Waiting for camera intrinsics to be sent
        camera_info = rospy.wait_for_message('/nerian_stereo/stereo_camera_info', StereoCameraInfo)

        K_matrix = camera_info.left_info.K
        self.Q = camera_info.Q

        # Storing the entrinsics for future calculations
        self.x_focal = K_matrix[0]
        self.y_focal = K_matrix[4]
        self.x_centre = K_matrix[2]
        self.y_centre = K_matrix[5]

        self.on = True

    # Turning off and deregistering from the rostopics
    def turn_off(self):
        self.on = False

    ### IMAGE PROCESSING ###
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
    def image_to_nerian(self, image_position: [int, int]) -> Point:
        # Pixel to millimeter conversion experiments
        # [476, 477] -> [1401, 535] = 600 mm

        pixel_to_millimeter = 0.645

        # Calculating depth in a simpler way
        position = Point()

        # Calculating depth using focal length and baseline and disparity
        position.z = 1.4
        position.x = 0.001 * (image_position[0] - 968) * pixel_to_millimeter
        position.y = 0.001 * (image_position[1] - 608) * pixel_to_millimeter

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

    ### CALLBACKS ###
    # Processing an image of the nerian
    def process_image(self, data: Image):
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
                fruit_locations.append(self.image_to_nerian(fruit[0], fruit[1]))
                cv.circle(image, (fruit[0], fruit[1]), 7, (255, 0, 255), -1)

        cv.namedWindow("nerian", cv.WINDOW_NORMAL)
        cv.resizeWindow("nerian", 800, 600)
        cv.imshow("nerian", image)
        cv.waitKey(1)

        return fruit_locations

# Defining an object that takes a stream of timestamped positions and predicts the next prediction
class crystal_ball:
    '''
    This class handles the prediction of kiwifruit position to account for the system latency.
    This class performs the same functionality as the 'motion_prediction' class, 
    it is recommended to use 'motion_prediction.'
    '''

    ### BASIC INTERFACING ###
    # Turning on and connecting up all the components
    def turn_on(self):
        # Starting up the input and output nodes of the system
        self.point_pub = rospy.Publisher("flame/predicted_position", Point, queue_size = 1)
        self.point_sub = rospy.Subscriber("/flame/perceived_position", PointStamped, self.predict_position)

        self.on = True

    # Closing down the system cleanly
    def turn_off(self):
        # Disconnecting all the cables
        self.point_pub.unregister()
        self.point_sub.unregister()

        self.on = False

    # Initialising the crystal ball
    def __init__(self):
        self.on = False

        # Declaring the lookahead time and predition method
        # 0- Velocity Average, 1- Velocity Smudge, 2- Acceleration Smudge, 3- Acceleration Smudge - Tuned, 4- No Prediction
        prediction_method = 3
        lookahead = 0.25

        # Declaring the prediction algorithm object
        self.fruit_predict = self.predict(prediction_method, lookahead)

        # Declaring an evaluation as well as a simulation
        self.fruit_evaluate = self.evaluate()
        self.fruit_simulate = self.simulate(lookahead)

        # Starting all the respective rostopic streams
        self.fruit_evaluate.start_stream()
 
    # Continuously runs a simulation with a specific motion until the q key is pressed
    def start_simulation(self):
        while True:
            future_pos, fruit_position = self.fruit_simulate.stream_simulation()

            # Run prediction model on simulation
            self.fruit_predict.predict_position(fruit_position)
            print("predicted: ", self.fruit_predict.predicted_pos)

            # Evaluate the prediction error
            self.fruit_evaluate.compare(fruit_position[:3], fruit_position[3], self.fruit_predict.predicted_pos)

            # Show visual of fruit
            self.fruit_evaluate.display(future_pos, self.fruit_predict.predicted_pos)

            # Run simulation at desired speed, exit on 'q'
            if (cv.waitKey(int((1000/self.fruit_simulate.simulation_sampling_frequency)/self.fruit_simulate.playback_speed)) & 0xFF == ord('q')):
                break

    # Callback for recieving fruit positions
    def predict_position(self, data: PointStamped):
        # Conver
        fruit_position = data.point
        reading_time = float(data.header.stamp.secs) + float(data.header.stamp.nsecs) / (10 ** 9) 

        self.fruit_predict.predict_position(fruit_position, reading_time)
        
        predicted_point = Point()

        predicted_point.x = self.fruit_predict.predicted_pos[0]
        predicted_point.y = self.fruit_predict.predicted_pos[1]
        predicted_point.z = self.fruit_predict.predicted_pos[2]

        self.point_pub.publish(predicted_point)

        # Calculate prediction error
        self.fruit_evaluate.compare(fruit_position, reading_time, self.fruit_predict.predicted_pos)

    # Handles the prediction model
    class predict:
        # Initialise
        def __init__(self, predict_method, lookahead):
            # Smudge Tuning Parameters
            self.gauss_size = 101
            self.GAUSSIAN_DIST = 0.35
            self.SMUDGE_ARRAY_SIZE = 301

            # Lookback Parameters
            self.SMUDGE_STORED_VALUES = 30
            self.DIMINISHING_FACTOR = 0.65

            # Acceleration and Velocity Parameters
            self.SLIDING_WINDOW_WIDTH_VEL = 8
            self.SLIDING_WINDOW_WIDTH_ACC = 12
            self.VELOCITY_GAIN = 0.4
            self.ACCELERATION_GAIN = 0.65

            self.PREDICT_METHOD = predict_method
            self.LOOKAHEAD = lookahead

            # Defining the expected number of readings per second
            self.samples_per_second = 60

            self.fruit_pos = np.zeros((self.samples_per_second, 4))
            self.fruit_vel = np.zeros((self.samples_per_second-1, 4))
            self.fruit_acc = np.zeros((self.samples_per_second-2, 4))
            self.fruit_jrk = np.zeros((self.samples_per_second-3, 4))
            self.predicted_pos = [0, 0, 0, 0]

            self.gauss = self.gaussuian_filter(self.gauss_size, self.GAUSSIAN_DIST, 0)

        # Apply prediction
        def predict_position(self, position: Point, reading_time: float):
            # Convert from a ros message to an array
            pos = [position.x, position.y, position.z, reading_time]

            # push onto shift register
            self.push_fruit_pos_sr(pos)

            #Velocity Average
            if (self.PREDICT_METHOD == 0):
                self.get_derivatives(1, 1)
                vel = self.recent_average(self.fruit_vel, 3)
                self.velocity_predict(vel, self.LOOKAHEAD)

            #Smudge Velocity
            elif (self.PREDICT_METHOD == 1):
                self.get_derivatives(1, self.SLIDING_WINDOW_WIDTH_VEL)
                self.smudge_predict(self.LOOKAHEAD, self.SMUDGE_STORED_VALUES, self.SMUDGE_ARRAY_SIZE, self.PREDICT_METHOD)

            #Smudge Acceleration Tuned
            elif (self.PREDICT_METHOD == 2):
                self.get_derivatives(2, self.SLIDING_WINDOW_WIDTH_VEL, self.SLIDING_WINDOW_WIDTH_ACC)
                self.smudge_predict(self.LOOKAHEAD, self.SMUDGE_STORED_VALUES, self.SMUDGE_ARRAY_SIZE, self.PREDICT_METHOD)

            #Smudge Acceleration Tuned
            elif (self.PREDICT_METHOD == 3):
                self.get_derivatives(2, self.SLIDING_WINDOW_WIDTH_VEL, self.SLIDING_WINDOW_WIDTH_ACC)
                self.smudge_predict(self.LOOKAHEAD, self.SMUDGE_STORED_VALUES, self.SMUDGE_ARRAY_SIZE, self.PREDICT_METHOD, self.VELOCITY_GAIN, self.ACCELERATION_GAIN)

            #No Prediction
            elif (self.PREDICT_METHOD == 4):
                self.predicted_pos = pos

        # Record new position
        def push_fruit_pos_sr(self, pos):
            self.fruit_pos = np.roll(self.fruit_pos, 1, 0)
            self.fruit_pos[0] = pos

        # Find derivatives of position with sliding windows
        def get_derivatives(self, n, wv, wa=1):

            # Number of derivatives
            if (n>0):
                # Velocity window size
                if wv == 1:
                    self.fruit_vel = self.find_derivative(self.fruit_pos)
                if wv > 1:
                    self.fruit_vel = self.find_derivative_sliding_window(self.fruit_pos, wv)
            if (n>1):
                # Acceleration window size
                if wa == 1:
                    self.fruit_acc = self.find_derivative(self.fruit_vel)
                if wa > 1:
                    self.fruit_acc = self.find_derivative_sliding_window(self.fruit_vel, wa)
            if (n>2):
                # Capacity to calculate Jerk, not used
                self.fruit_jrk = self.find_derivative(self.fruit_acc)  

        # Calculate derivative array of given array
        def find_derivative(self, arr):

            # Initialise output array
            derr_arr = np.zeros((len(arr)-1, 4))

            prev_el = arr[0]
            i = 0
            # Calculate output array
            for el in arr[1:]:
                # Find change in time
                dt = prev_el[3]-el[3]
                if (dt==0):
                    dt = 1/self.samples_per_second

                # Change in element over change in time
                derr_arr[i][0] = (prev_el[0] - el[0])/dt
                derr_arr[i][1] = (prev_el[1] - el[1])/dt
                derr_arr[i][2] = (prev_el[2] - el[2])/dt
                derr_arr[i][3] = el[3]
                prev_el = el
                i += 1
            return derr_arr
        
        # Calculate derivative of array with sliding window method
        def find_derivative_sliding_window(self, arr, w):

            # Initialise Output Array
            derr_arr = np.zeros((len(arr)-w, 4))

            # For possible windows in array
            for i in range(len(arr[1:len(arr)-w])):

                dt = 0
                derr_arr[i] = [0, 0, 0, 0]
                # Find average timestep over window
                for j in range(w):
                    dt += (arr[i+j][3] - arr[i+j+1][3])/w
                if (dt==0):
                    dt = 1/self.samples_per_second
                
                # Find average derivative over array
                for k in range(w):
                    derr_arr[i][0] += (arr[i+k][0] - arr[i+k+1][0])/(dt*w)
                    derr_arr[i][1] += (arr[i+k][1] - arr[i+k+1][1])/(dt*w)
                    derr_arr[i][2] += (arr[i+k][2] - arr[i+k+1][2])/(dt*w)
                derr_arr[i][3] = arr[int(i+(w/2))][3]
            return derr_arr  
        
        # Apply Smudge prediction method
        def smudge_predict(self, lookahead, n, sz, method, velocity_gain=1, acceleration_gain=1):
            # Find derrivatives required by prediction method
            if method == 1:
                pred = self.interpolate_vel(lookahead, n)
            elif (method == 2) or (method == 3):
                pred = self.interpolate_acc(lookahead, n, velocity_gain, acceleration_gain)
            
            # Initialise the Smudge
            smudge = np.zeros((sz, sz))

            # Place most recent prediction in the center of the smudge
            c = int((sz-self.gauss.shape[0])/2)
            smudge[c:c+self.gauss.shape[0], c:c+self.gauss.shape[1]] = self.gauss
            x_init = pred[0][0]
            y_init = pred[0][1]

            # Place remaining predictions on the Smudge
            for p in range(len(pred)-1):
                x_offs = int((pred[p+1][0] - x_init)*2000)
                y_offs = int((pred[p+1][1] - y_init)*2000)
                if (abs(x_offs) < (sz-self.gauss_size)/2) & (abs(y_offs) < (sz-self.gauss_size)/2):
                    smudge[c+x_offs:c+x_offs+self.gauss.shape[0], c+y_offs:c+y_offs+self.gauss.shape[1]] += (self.DIMINISHING_FACTOR**(p+1))*self.gauss
                else:
                    print('omitted: ', p)

            # Find the highest value in the smudge calculate predicted position
            predicted_indx = np.where(smudge == smudge.max())
            predicted_position = [x_init + ((predicted_indx[0][0]-((sz-1)/2))/2000), 
                                y_init + ((predicted_indx[1][0]-((sz-1)/2))/2000), 
                                self.fruit_pos[0][2], self.fruit_pos[0][3] + lookahead]
            self.predicted_pos = predicted_position
            smudge = cv.circle(smudge, (int(predicted_indx[1][0]), int(predicted_indx[0][0])), 3, (0, 0, 0), 1)

            # cv.imshow('smudge', smudge)
            # cv.waitKey(1)

        # Calculate prediction array assuming acceleration to be constant
        def interpolate_acc(self, lookahead_time, n, p, q):
            pred = np.zeros((n, 4))
            lookahead_ts = self.fruit_pos[0][3] + lookahead_time
            for i in range(n):
                dt = lookahead_ts - self.fruit_pos[i][3]
                pred[i][0] = self.fruit_pos[0][0] + (p*self.fruit_vel[i][0]*dt) + 0.5*(q*self.fruit_acc[i][0]*(dt**2))
                pred[i][1] = self.fruit_pos[0][1] + (p*self.fruit_vel[i][1]*dt) + 0.5*(q*self.fruit_acc[i][1]*(dt**2))
                pred[i][2] = self.fruit_pos[0][2] + (self.fruit_vel[i][2]*dt) + 0.5*(self.fruit_acc[i][2]*(dt**2))
            return pred
        
        # Calculate prediciton array assuming velocity to be constant
        def interpolate_vel(self, lookahead_time, n):
            pred = np.zeros((n, 4))
            lookahead_ts = self.fruit_pos[0][3] + lookahead_time
            for i in range(n):
                dt = lookahead_ts - (self.fruit_pos[i][3])
                pred[i][0] = self.fruit_pos[0][0] + (self.fruit_vel[i][0]*dt)
                pred[i][1] = self.fruit_pos[0][1] + (self.fruit_vel[i][1]*dt)
                pred[i][2] = self.fruit_pos[0][2] + (self.fruit_vel[i][2]*dt)
            return pred
        
        # Predict position assuming a linear path
        def velocity_predict(self, vel, lookahead_time):
            predicted_pos = [0,0,0,0]
            pos = self.fruit_pos[0]
            predicted_pos[0] = pos[0] + vel[0]*lookahead_time
            predicted_pos[1] = pos[1] + vel[1]*lookahead_time
            predicted_pos[2] = pos[2] + vel[2]*lookahead_time
            predicted_pos[3] = pos[3] + lookahead_time
            self.predicted_pos = predicted_pos

        # Initialise Gaussian array
        def gaussuian_filter(self, kernel_size, sigma=0.5, muu=0):
            x, y = np.meshgrid(np.linspace(-1, 1, kernel_size), np.linspace(-1, 1, kernel_size))
            dst = np.sqrt(x**2+y**2)
            gauss = np.exp(-((dst-muu)**2 / (2.0 * sigma**2)))
            return gauss
        
        # Find average of arr over recent n readings
        def recent_average(self, arr, n):
            vec = [0, 0, 0, 0] #xyzt
            for i in range(n):
                vec[0] += (arr[i][0])/n
                vec[1] += (arr[i][1])/n
                vec[2] += (arr[i][2])/n
            vec[3] = arr[0][3]
            return vec

    # Handles the evaluation of fruit tracking
    class evaluate:
        # Initialise
        def __init__(self):
            self.empty = np.zeros((1000, 1000, 3))
            self.predicted_positions = [[0, 0, 0, 0]]
            self.error_stats = []
            self.flag_c = 0
            self.first_flag = 0
            self.first_position = [0, 0, 0, 0]

        # Start the stream for the prediction error
        def start_stream(self):
            self.predict_pub = rospy.Publisher("flame/prediction_error", PointStamped, queue_size = 1)

        # Compares streams of position and predicted_position
        def compare(self, actual_position: Point, time, predicted_position):
            # Converting from the ROS message to an array
            position = [actual_position.x, actual_position.y, actual_position.z, time]

            # Check for NaN values
            predicted_position = self.check_predicted_position(position, predicted_position)

            # Compare position to its respective prediction
            self.calculate_error(position, predicted_position)

        # Visualise fruit position vs its prediction
        def display(self, position, predicted_position, DISPLAY_MOVEMENT_GAIN = 5000):
            movement_gain = DISPLAY_MOVEMENT_GAIN

            # Wait for 30 readings to define 
            if (self.first_flag == 0):
                self.flag_c += 1
                if (self.flag_c > 30):
                    self.first_position = position
                    self.first_flag = 1

            # Display both positions from the perspective below
            im_below = np.zeros((500, 500, 3))
            im_below = cv.circle(im_below, (250+int(self.first_position[0] + movement_gain*position[0] ), 250+int(self.first_position[1] + movement_gain*position[1])), 5, (255, 255, 255), -1)
            im_below = cv.circle(im_below, (250+int(self.first_position[0] + movement_gain*predicted_position[0]), 250+int(self.first_position[1] + movement_gain*predicted_position[1])), 5, (0, 0, 255), -1)
            cv.imshow('below', im_below)

            # Display both positions from a perspective to the side
            im_side = np.zeros((500, 500, 3))
            im_side = cv.circle(im_side, (250+int(self.first_position[0] + movement_gain*position[0]), 250+int(self.first_position[2] + movement_gain*position[2])), 5, (255, 255, 255), -1)
            im_side = cv.circle(im_side, (250+int(self.first_position[0] + movement_gain*predicted_position[0]), 250+int(self.first_position[2] + movement_gain*predicted_position[2])), 5, (0, 0, 255), -1)
            cv.imshow('side', im_side)

        # Chack for NaN values
        def check_predicted_position(self, pos, predicted_pos):
            for i in range(len(predicted_pos)):
                if (np.isnan(predicted_pos[i]) == 1):
                    predicted_pos[i] = pos[i]
                    print('Warning: NaN detected')
            return predicted_pos
        
        # Find error between position and prediction
        def calculate_error(self, pos, predicted_pos):
            del_idx = []
            error = [0, 0, 0, 0]

            # Record incoming predictions
            self.predicted_positions = np.concatenate((self.predicted_positions, [predicted_pos]))

            time_diff = 1
            nearest_idx = 0
            # Match time stamp of newest position to its respective prediction 
            for p in range(len(self.predicted_positions)):

                # Find the prediction with the clossest time stamp
                if (abs(pos[3] - self.predicted_positions[p][3]) < time_diff):
                    time_diff = abs(pos[3] - self.predicted_positions[p][3])
                    nearest_idx = p

                # Record all expired predictions
                if (pos[3] > self.predicted_positions[p][3]):
                    del_idx.append(p)
                    
            # Find Prediction Error
            error[0] = pos[0] - self.predicted_positions[nearest_idx][0]
            error[1] = pos[1] - self.predicted_positions[nearest_idx][1]
            error[2] = pos[2] - self.predicted_positions[nearest_idx][2]
            error[3] = pos[3] - self.predicted_positions[nearest_idx][3]
            abs_error = math.sqrt((error[0]**2) + (error[1]**2) + (error[2]**2))

            # Delete expired predictions
            self.predicted_positions = np.delete(self.predicted_positions, del_idx, 0)

            # Publish prediction error
            pub_prediction = PointStamped()

            pub_prediction.point.x = float(error[0])
            pub_prediction.point.y = float(error[1])
            pub_prediction.point.z = float(error[2])

            pub_prediction.header.stamp.secs = int(error[3])
            pub_prediction.header.stamp.nsecs = int((error[3] % 1) * 10 ** 9)

            self.predict_pub.publish(pub_prediction)

    # Simulate a fruit swinging 
    class simulate:
        # Initialise
        def __init__(self, lookahead):
            self.r = 0.04
            self.theta = 0
            self.theta_0 = 0
            self.phi = math.pi
            self.A_x = 0.02
            self.f_th = 1
            self.lookahead = lookahead
            self.simulation_sampling_frequency = 50
            self.playback_speed = 1
            self.A_th = math.asin(self.A_x/self.r)
            self.sphr_pos = [self.r, self.theta, self.phi]
            self.cart_pos = [0,0,0]
            self.fruit_p = [0,0,0,0]
            self.phase_th = 0
            self.time_stamp = 0
            self.fruit_SR = np.zeros((int((self.simulation_sampling_frequency*self.lookahead)+1), 4))

        # Generate next simulated fruit position
        def inc_fruit_position(self):
            self.phase_th += self.f_th*2*math.pi/self.simulation_sampling_frequency
            self.theta = self.theta_0 + self.A_th*math.sin(self.phase_th)

            if (self.phase_th >= 2*math.pi):
                self.phase_th -= 2*math.pi

            self.time_stamp += 1/self.simulation_sampling_frequency
            self.sphr_pos = [self.r, self.theta, self.phi]
            self.sphr_to_cart()

        # Convert to xyz
        def sphr_to_cart(self):
            self.cart_pos[0] = self.sphr_pos[0] * math.sin(self.sphr_pos[1]) * math.cos(self.sphr_pos[2])
            self.cart_pos[1] = self.sphr_pos[0] * math.sin(self.sphr_pos[1]) * math.sin(self.sphr_pos[2])
            self.cart_pos[2] = self.sphr_pos[0] * math.cos(self.sphr_pos[1])

        # Convert to xyzt
        def gen_fruit_pos(self):
            self.fruit_p = np.concatenate((self.cart_pos, [self.time_stamp]))

        # Hold Positions until future position is known for display purposes
        def send_delayed(self):
            self.fruit_SR = np.roll(self.fruit_SR, 1, 0)
            self.fruit_SR[0] = self.fruit_p
            return self.fruit_SR[0], self.fruit_SR[-1]
        
        # Run simulation
        def stream_simulation(self):
            self.inc_fruit_position()
            self.gen_fruit_pos()
            future_pos, sim = self.send_delayed()
            return future_pos, sim

# Defining an alternative object that takes a stream of timestamped positions and predicts the next prediction
class motion_estimation:
    '''
    This class handles the prediction of kiwifruit position to account for the system latency
    using variations of kalman filters or particle filters.
    This class has similar functionality to the 'crystal_ball' class.
    '''
    ### BASIC INTERFACING ###
    def __init__(self, estimation_method):
        if estimation_method == 'particle filter 1':
            self.kf = self.particle_filter()
        elif estimation_method == 'particle filter 2':
            self.kf = self.particle_filter_fewer_states()
        elif estimation_method == 'kalman filter 1':
            self.kf = self.kalman_filter_constant_acceleration()
        elif estimation_method == 'kalman filter 2':
            self.kf = self.kalman_filter_cranks_EKF()

    # Turning on and connecting up all the components
    def turn_on(self):
        # Starting up the input and output nodes of the system
        self.point_pub = rospy.Publisher("flame/predicted_position", Point, queue_size = 1)
        self.point_sub = rospy.Subscriber("/flame/perceived_position", PointStamped, self.predict_position)

        self.on = True

    # Closing down the system cleanly
    def turn_off(self):
        # Disconnecting all the cables
        self.point_pub.unregister()
        self.point_sub.unregister()

        self.on = False

    def predict_position(self, data: PointStamped):
        position = data.point
        measurement = np.matrix([[position.x], [position.y], [position.z]])
        pos = self.kf.run_filter(measurement, 0)
        self.publish(pos[:3])

    def publish(self, pos):
        predicted_point = Point()
        predicted_point.x = pos[0]
        predicted_point.y = pos[1]
        predicted_point.z = pos[2]
        
        self.point_pub.publish(predicted_point)
    
    class kalman_filter_constant_acceleration:
        '''
        This class uses a kalman filter to predict the position of the kiwifruit.
        The filter assumes a constant acceleration model and does not model the real motion well.
        This filter does not work in simulation.
        '''
        
        def __init__(self):
            R_std = 0.002
            Q_std = 0.01

            self.kalmfilt = self.tracker(R_std, Q_std)

            # File header
            self.f = open("Kalman_filter_tracking.csv", "w")
            self.f.write("count,measurement x,measurement y,measurement z,kalman gain x,kalman gain y,kalman gain y,state x,state x.,state x..,state y,state y.,state y..,state z,state z.,state z..,outx,outy,outz,covariance x,covariance y,covariance z\n")
            self.f.close()
            self.count = 0

        def tracker(self, R_std, Q_std):
            kf = KalmanFilter(dim_x=9, dim_z=3)

            dt = 1.0/60   # time step

            kf.F = np.array([[1, dt, 0.5*dt**2, 0, 0, 0, 0, 0, 0], 
                             [0, 1, dt, 0, 0, 0, 0, 0, 0],
                             [0, 0, 1, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 1, dt, 0.5*dt**2, 0, 0, 0],
                             [0, 0, 0, 0, 1, dt, 0, 0, 0],
                             [0, 0, 0, 0, 0, 1, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 1, dt, 0.5*dt**2],
                             [0, 0, 0, 0, 0, 0, 0, 1, dt],
                             [0, 0, 0, 0, 0, 0, 0, 0, 1],])
            
            kf.u = 0
            kf.R = np.eye(3) * R_std**2
            kf.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0], 
                            [0, 0, 0, 1, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 1, 0, 0]])
            
            kf.Q = Q_discrete_white_noise(3, dt=dt, var=Q_std, block_size=3)

            # Interesting - inconsistent with sources
            # q = Q_discrete_white_noise(dim=2, dt=1, var=0.001)
            # kf.Q=block_diag(q,q)
            # print("Q:", kf.Q)
            
            kf.x = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0]]).T
            kf.P = np.eye(9) * 5.
            return kf
            
        def run_filter(self, measurement, SIM):
            z = np.asarray(measurement)
            self.kalmfilt.predict()
            self.kalmfilt.update(z)

            self.print_to_file(measurement)
            
            # return output position state
            return [self.kalmfilt.x[0,0], self.kalmfilt.x[3,0], self.kalmfilt.x[6,0]]
        
        def print_to_file(self, measurement):
            # print("State update ", self.state_vector)
            self.f = open("Kalman_filter_tracking.csv", "a")
            self.f.write(str(self.count))
            self.f.write(',')
            self.count += 1

            self.f.write(str(measurement[0,0]))
            self.f.write(',')
            self.f.write(str(measurement[1,0]))
            self.f.write(',')
            self.f.write(str(measurement[2,0]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.K[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.K[1,1]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.K[2,2]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.x[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[1,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[2,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[3,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[4,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[5,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[6,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[7,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[8,0]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.P[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.P[1,1]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.P[2,2]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.y[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.y[1,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.y[2,0]))
            self.f.write('\n')

            self.f.close()

    class kalman_filter_cranks_EKF:
        '''
        This filter uses the Extended Kalman Filter to better model the non-linear motion of the kiwifruit.
        This filter may not work in Simulation
        '''
        def __init__(self):
            R_std = 0.002
            Q_std = 0.01

            self.crank_offset = 0.08
            self.length = 0.38

            self.kalmfilt = self.tracker(R_std, Q_std)

            # File header
            self.f = open("Kalman_filter_tracking.csv", "w")
            self.f.write("count,measurement x,measurement y,measurement z,kalman gain x,kalman gain y,kalman gain y,state x,state x.,state x..,state y,state y.,state y..,state z,state z.,state z..,outx,outy,outz,covariance x,covariance y,covariance z\n")
            self.f.close()
            self.count = 0

        def tracker(self, R_std, Q_std):
            ekf = ExtendedKalmanFilter(dim_x=11, dim_z=3)

            self.dt = 1.0/60   # time step

            ekf.x = np.array([[0, 0, 0.02, 0, 1, 0, 0, 0.02, 0, 1, 0]]).T

            ekf.F = self.calculate_F()
            
            ekf.u = 0
            ekf.R = np.eye(3) * R_std**2
            ekf.H = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
            
            ekf.Q = Q_discrete_white_noise(3, dt=self.dt, var=Q_std, block_size=3)

            # Interesting - inconsistent with sources
            # q = Q_discrete_white_noise(dim=2, dt=1, var=0.001)
            # kf.Q=block_diag(q,q)
            # print("Q:", kf.Q)
            
            ekf.P = np.eye(11) * 5.
            return ekf
            
        def run_filter(self, measurement, SIM):
            z = np.asarray(measurement)
            self.kalmfilt.predict()
            self.kalmfilt.update(z, )

            self.print_to_file(measurement)
            
            # return output position state
            return [self.kalmfilt.x[1,0], self.kalmfilt.x[6,0], self.kalmfilt.x[10,0]]
        
        def calculate_F(self):
            rx = self.kalmfilt.x[2,0]
            thx = self.kalmfilt.x[3,0]
            dx_rx = math.cos(rx) - ((rx*math.cos(thx))*(rx*math.sin(thx) - self.crank_offset))/(math.sqrt((self.length**2) - (self.crank_offset - rx*math.sin(thx))**2))
            ry = self.kalmfilt.x[7,0]
            thy = self.kalmfilt.x[8,0]
            dx_ry = math.cos(ry) - ((ry*math.cos(thy))*(ry*math.sin(thy) - self.crank_offset))/(math.sqrt((self.length**2) - (self.crank_offset - ry*math.sin(thy))**2))
                                                                                                      
            F = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                          [1, 0, dx_rx, -rx*math.sin(thx), 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 1, self.dt, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, 0, dx_ry, -ry*math.sin(thy), 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
            
            return F

        def print_to_file(self, measurement):
            # print("State update ", self.state_vector)
            self.f = open("Kalman_filter_tracking.csv", "a")
            self.f.write(str(self.count))
            self.f.write(',')
            self.count += 1

            self.f.write(str(measurement[0,0]))
            self.f.write(',')
            self.f.write(str(measurement[1,0]))
            self.f.write(',')
            self.f.write(str(measurement[2,0]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.K[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.K[1,1]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.K[2,2]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.x[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[1,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[2,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[3,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[4,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[5,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[6,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[7,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[8,0]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.P[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.P[1,1]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.P[2,2]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.y[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.y[1,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.y[2,0]))
            self.f.write('\n')

            self.f.close()
            
        def run_filter(self, measurement, SIM):
            z = np.asarray(measurement)
            self.kalmfilt.F = self.calculate_F()
            self.kalmfilt.predict()
            self.kalmfilt.update(z)

            self.print_to_file(measurement)
            
            # return output position state
            return [self.kalmfilt.x[1,0], self.kalmfilt.x[6,0], self.kalmfilt.x[10,0]]
        
        def calculate_F(self):
            rx = self.kalmfilt.x[2,0]
            thx = self.kalmfilt.x[3,0]
            dx_rx = math.cos(rx) - ((rx*math.cos(thx))*(rx*math.sin(thx) - self.crank_offset))/(math.sqrt((self.length**2) - (self.crank_offset - rx*math.sin(thx))**2))
            ry = self.kalmfilt.x[7,0]
            thy = self.kalmfilt.x[8,0]
            dx_ry = math.cos(ry) - ((ry*math.cos(thy))*(ry*math.sin(thy) - self.crank_offset))/(math.sqrt((self.length**2) - (self.crank_offset - ry*math.sin(thy))**2))
                                                                                                      
            F = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                          [1, 0, dx_rx, -rx*math.sin(thx), 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 1, self.dt, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, 0, dx_ry, -ry*math.sin(thy), 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
            
            return F
        
        def init_F(self):
            rx = 0.02
            thx = 0
            dx_rx = math.cos(rx) - ((rx*math.cos(thx))*(rx*math.sin(thx) - self.crank_offset))/(math.sqrt((self.length**2) - (self.crank_offset - rx*math.sin(thx))**2))
            ry = 0.02
            thy = 0
            dx_ry = math.cos(ry) - ((ry*math.cos(thy))*(ry*math.sin(thy) - self.crank_offset))/(math.sqrt((self.length**2) - (self.crank_offset - ry*math.sin(thy))**2))
                                                                                                      
            F = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                          [1, 0, dx_rx, -rx*math.sin(thx), 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 1, self.dt, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, 0, dx_ry, -ry*math.sin(thy), 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
            
            return F

        def print_to_file(self, measurement):
            # print("State update ", self.state_vector)
            self.f = open("Kalman_filter_tracking.csv", "a")
            self.f.write(str(self.count))
            self.f.write(',')
            self.count += 1

            self.f.write(str(measurement[0,0]))
            self.f.write(',')
            self.f.write(str(measurement[1,0]))
            self.f.write(',')
            self.f.write(str(measurement[2,0]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.K[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.K[1,1]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.K[2,2]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.x[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[1,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[2,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[3,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[4,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[5,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[6,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[7,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[8,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[9,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.x[10,0]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.P[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.P[1,1]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.P[2,2]))
            self.f.write(',')

            self.f.write(str(self.kalmfilt.y[0,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.y[1,0]))
            self.f.write(',')
            self.f.write(str(self.kalmfilt.y[2,0]))
            self.f.write('\n')

            self.f.close()
    
    class particle_filter:
        '''
        This filter uses a particle filter to model the motion of a kiwifruit and predict its position.
        This filter works in simulation but has not been implimented on the UR5.
        '''
        def __init__(self):
            self.init_flag = 0
            self.N = 10000
            self.dt = 1/60
            plt.figure()
            plt.ion()

            # File header
            self.f = open("Kalman_filter_tracking.csv", "w")
            self.f.write("count,measurement x,measurement y,measurement z,x,y,z,thx,rx,x0,thy,ry,y0,thx.,thy.\n")
            self.f.close()
            self.count = 0

        # Create a gaussian spread of N particles x,y,z,x0,y0,rx,thx,alpx,ry,thy,alpy
        def create_gaussian_particles(self, z, N):
            mean = np.asarray(z, dtype='float')
            print(mean)
            std = np.array([0.3, 0.3, 0.2])
            self.l = 0.38
            self.o = 0.08

            particles = np.empty((N, 12), dtype='double')
            particles[:, 0] = float(mean[0]) + (np.random.randn(N) * std[0])      #x
            particles[:, 1] = float(mean[1]) + (np.random.randn(N) * std[1])      #y
            particles[:, 2] = float(mean[2]) + (np.random.randn(N) * std[2])      #z
            particles[:, 3] = np.random.uniform(low=0.0, high=2*math.pi, size=N)       #thx
            particles[:, 4] = np.random.uniform(low=0.01, high=0.05, size=N)            #rx
            particles[:, 5] = particles[:, 4]*np.cos(particles[:, 3]) + np.sqrt(self.l**2 - (particles[:, 4]*np.sin(particles[:, 3]) - self.o)**2) - particles[:, 0] #x0
            print(particles[:, 5])
            particles[:, 6] = np.random.uniform(low=0.0, high=2*math.pi, size=N)       #thy
            particles[:, 7] = np.random.uniform(low=0.01, high=0.05, size=N)            #ry
            particles[:, 8] = particles[:, 7]*np.cos(particles[:, 6]) + np.sqrt(self.l**2 - (particles[:, 7]*np.sin(particles[:, 6]) - self.o)**2) - particles[:, 1] #y0
            particles[:, 9] = np.random.uniform(low=0.0, high=3.0, size=N)       #alpx
            particles[:, 10] = np.random.uniform(low=0.0, high=3.0, size=N)      #alpy
            particles[:, 11] = np.ones(N)/N #weight
            
            return particles

        def run_filter(self, measurement, SIM):
            z = np.asarray(measurement, dtype='float')
            if (self.init_flag):
                R = 0.01
                self.update(z, R)
                mean, var = self.estimate_state()
                print(mean)
                self.print_to_file(z, mean)
                self.evaluate(mean, z)
                self.predict()
            else:
                print("initial guess", z)
                self.particles = self.create_gaussian_particles(z, self.N)
                self.init_flag = 1
                mean = z

            return mean

        def update(self, z, R):
            distance = np.linalg.norm(self.particles[:, 0:3] - z, axis=1)
            # print("distance:", distance)
            self.particles[:, 11] *= scipy.stats.norm(loc=0, scale=math.sqrt(3*(R**2))).pdf(distance)
            # print("weights:", self.weights)
            self.particles[:, 11] += 1.e-300 # avoid round-off to zero
            self.particles[:, 11] /= sum(self.particles[:, 11]) # normalize

        def predict(self):
            self.particles[:, 0] = self.particles[:, 4]*np.cos(self.particles[:, 3]) + np.sqrt(self.l**2 - (self.particles[:, 4]*np.sin(self.particles[:, 3]) - self.o)**2) + self.particles[:, 5]      #x=rxcos(thx) + sqrt(l^2 - (rxsin(thx)-o)^2) + x0
            self.particles[:, 1] = self.particles[:, 7]*np.cos(self.particles[:, 6]) + np.sqrt(self.l**2 - (self.particles[:, 7]*np.sin(self.particles[:, 6]) - self.o)**2) + self.particles[:, 8]      #y=rycos(thy) + sqrt(l^2 - (rysin(thy)-o)^2) + y0
            self.particles[:, 3] += self.particles[:, 9]*self.dt                      #thx += alpx
            self.particles[:, 6] += self.particles[:, 10]*self.dt                     #thy += alpy
                                                                                                                        # x, y, z,    thx,    rx,   x0,     thy,   ry,  y0,   thx.  thy.
            self.particles[:, :11] = np.multiply(self.particles[:, :11], (1 - (np.multiply(np.random.randn(self.N, 11), [0, 0, 0.001, 0.01, 0.001, 0.02, 0.01, 0.001, 0.02, 0.05, 0.05]))))

            self.constrain_state_vars()

        def constrain_state_vars(self):
            #x,y,z,thx,rx,x0,thy,ry,y0,thx.,thy.
            l_bound = [0, 0, 0, 0, 0.04, 0, 0, 0, 0, 0, 0]
            u_bound = [0.5, 0.5, 0.5, 2*math.pi, 0.08, 1, 2*math.pi, 0.08, 1, 2*math.pi, 2*math.pi]
            self.particles[:, :11] = np.clip(self.particles[:, :11], l_bound, u_bound)

        def neff(self):
            return np.sum(np.square(self.particles[:, 11]))

        def resample(self):
            N = len(self.particles[:, 11])
            indexes = np.zeros(N, dtype=int)
            # take int(N*w) copies of each weight
            w = np.asarray(self.particles[:, 11])
            num_copies = (N*w).astype(int)
            k = 0
            for i in range(N):
                for _ in range(num_copies[i]): # make n copies
                    indexes[k] = i
                    k += 1
                    
            # use multinormial resample on the residual to fill up the rest.
            residual = w - num_copies # get fractional part
            residual /= sum(residual) # normalize
            cumulative_sum = np.cumsum(residual)
            cumulative_sum[-1] = 1. # ensures sum is exactly one
            indexes[k:N] = np.searchsorted(cumulative_sum, np.random.random(N-k))

            self.particles[:] = self.particles[indexes]

        def estimate_state(self):

            """returns mean and variance of the weighted particles"""
            pos = self.particles[:, 0:11]
            mean = np.average(pos, weights=self.particles[:, 11], axis=0)
            var = np.average((pos - mean)**2, weights=self.particles[:, 11], axis=0)
            return mean, var

        def evaluate(self, pos, z):
            # if (self.neff() > 0.5):
                # self.cull_by_weight()
            # print("neff", self.neff())
            self.plot(pos, z)

            self.resample()

        def plot(self, pos, z):
            plt.clf()
            plt.scatter(self.particles[:, 0], self.particles[:, 1], label='Particles', color='blue', alpha=(self.particles[:, 11]/np.max(self.particles[:, 11])))
            plt.scatter(pos[0], pos[1], label='Estimate', color='red')
            plt.scatter(z[0], z[1], label='Estimate', color='black')

            plt.xlim(0.25, 0.45)
            plt.ylim(0.2, 0.4)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Particle Positions')
            plt.show()
            plt.pause(0.01)

        def print_to_file(self, measurement, vars):
            self.f = open("Kalman_filter_tracking.csv", "a")
            self.f.write(str(self.count))
            self.f.write(',')
            self.count += 1

            self.f.write(str(measurement[0]))
            self.f.write(',')
            self.f.write(str(measurement[1]))
            self.f.write(',')
            self.f.write(str(measurement[2]))
            self.f.write(',')

            self.f.write(str(vars[0]))
            self.f.write(',')
            self.f.write(str(vars[1]))
            self.f.write(',')
            self.f.write(str(vars[2]))
            self.f.write(',')
            self.f.write(str(vars[3]))
            self.f.write(',')
            self.f.write(str(vars[4]))
            self.f.write(',')
            self.f.write(str(vars[5]))
            self.f.write(',')
            self.f.write(str(vars[6]))
            self.f.write(',')
            self.f.write(str(vars[7]))
            self.f.write(',')
            self.f.write(str(vars[8]))
            self.f.write(',')
            self.f.write(str(vars[9]))
            self.f.write(',')
            self.f.write(str(vars[10]))
            self.f.write('\n')
            self.f.close()

    class particle_filter_fewer_states:
        '''
        This particle filter attempts to model a swinging kiwiruit.
        This filter works in simulation but is not implimented on the UR5. 
        '''
        def __init__(self):
            self.init_flag = 0

            self.lookahead = 0.3
            self.rate = 60
            self.measurement_pipeline_for_sim = -1*np.ones((int(self.rate*self.lookahead)+1, 3))

            # Tuning
            self.N = 2000
            self.dt = 1/60
            self.R = 0.005
                                       # x,     y,      z,      thx,    ax,     thy,    ay,     thx.    thy.
            self.fluff_array = np.array([0.0005,0.0005, 0.002,  0.3,    0.002,  0.3,    0.002,  0.1,   0.1])
            self.show_future_prediction = True
            self.show_current_state_estimation = True

            plt.figure()
            plt.ion()

            # File header
            self.f = open("Kalman_filter_tracking.csv", "w")
            self.f.write("count,measurement x,measurement y,measurement z,x,y,z,thx,ax,thy,ay,thx.,thy.\n")
            self.f.close()
            self.count = 0

            # Create a window
            cv.namedWindow("Particle_Filter_Tuning")
            cv.resizeWindow("Particle_Filter_Tuning", 100, 200)

            self.window_limits = [0.25, 0.45, 0.2, 0.4]

            # Create trackbars for each element in fluff_array
            state_labels = ["x", "y", "z", "thx", "ax", "thy", "ay", "thx.", "thy."]
            self.create_sliders(state_labels, self.fluff_array, 10000.0, "fluff_array")

            # Create trackbars for changing the window limits
            window_labels = ["x min", "x max", "y min", "y max"]
            self.create_sliders(window_labels, self.window_limits, 100.0, "window")

        # Defines the process flow of the particle filter
        def run_filter(self, measurement, SIM):
            if SIM:
                measurement, future_measurement = self.delay_measurements(np.asarray(measurement, dtype='float'), lookahead=self.lookahead, rate=self.rate)
            z = np.asarray(measurement, dtype='float')
            if (self.init_flag):
                # Extrapolate particles to lookahead
                if self.show_future_prediction:
                    self.predict_lookahead(self.lookahead)
                    if SIM:
                        self.plot(future_measurement[0], future_measurement[1], 1, "pink", 0)

                # Predict one step in the future
                self.predict()

                # Update position based on measurement
                mean = self.update(z, self.R) 
            elif (SIM) & (measurement[0] == -1):
                mean = z
            else:
                # Initialise points centred around first reading
                print("\nFilter starting")
                print("initial guess", z)
                self.particles = self.create_gaussian_particles(z, self.N)
                self.init_flag = 1
                mean = z

            key = cv.waitKey(1) & 0xFF
            if key == ord('r'):
                self.init_flag = 0
            elif key == ord('p'):
                cv.waitKey(0)
            elif key == ord('q'):
                return [-999]
                
            return mean

        # Callback for updating state arrays with trackbar
        def trackbar_callback(self, value, index, norm, arr):
            normalized_value = float(value) / norm
            if arr == "window":
                self.window_limits[index] = normalized_value
            elif arr == "fluff_array":
                self.fluff_array[index] = normalized_value

        # Creates a series of sliders for altering kalman filter tuning parameters
        def create_sliders(self, labels, arr, norm, name):
            for i, value in enumerate(arr):
                trackbar_name = labels[i]
                cv.createTrackbar(trackbar_name, "Particle_Filter_Tuning", int(value*norm), int(norm), lambda val, idx=i: self.trackbar_callback(val, idx, norm, name))
        
        # Create a gaussian spread of N particles x,y,z,x0,y0,rx,thx,alpx,ry,thy,alpy
        def create_gaussian_particles(self, z, N):
            mean = np.asarray(z, dtype='float')
            std = np.array([0.3, 0.3, 0.2])
            self.l = 0.38
            self.o = 0.08

            particles = np.empty((N, 10), dtype='double')
            particles[:, 0] = float(mean[0]) + (np.random.randn(N) * std[0])      #x
            particles[:, 1] = float(mean[1]) + (np.random.randn(N) * std[1])      #y
            particles[:, 2] = float(mean[2]) + (np.random.randn(N) * std[2])      #z
            particles[:, 3] = np.random.uniform(low=0.0, high=2*math.pi, size=N)       #thx
            particles[:, 4] = np.random.uniform(low=0.01, high=0.1, size=N)            #ax
            particles[:, 5] = np.random.uniform(low=0.0, high=2*math.pi, size=N)       #thy
            particles[:, 6] = np.random.uniform(low=0.01, high=0.1, size=N)            #ay
            particles[:, 7] = np.random.uniform(low=0.0, high=2*math.pi, size=N)       #alpx
            particles[:, 8] = np.random.uniform(low=0.0, high=2*math.pi, size=N)      #alpy
            particles[:, 9] = np.ones(N)/N #weight
            
            return particles
        
        # 
        def delay_measurements(self, measurement, lookahead, rate):
            len = int(lookahead*rate)
            self.measurement_pipeline_for_sim = np.roll(self.measurement_pipeline_for_sim, shift=1, axis=0)
            self.measurement_pipeline_for_sim[0, :] = measurement
            return self.measurement_pipeline_for_sim[len], measurement
        
        def predict_lookahead(self, lookahead):
            prediction = self.extrapolate_state(lookahead)
            m,v = self.estimate_state(prediction)
            self.plot(prediction[:, 0], prediction[:, 1], prediction[:, 9], 'green', 1)
            self.plot(m[0], m[1], 1, 'orange', 0)

        def predict(self):
            self.particles = self.extrapolate_state(1/60)
            self.constrain_state_variables()

        def update(self, z, R):
            self.compute_weights(z, R)
            mean, var = self.estimate_state(self.particles)
            
            print(f"\rx: {mean[0]:.4f},{var[0]:.4f} y: {mean[1]:.4f},{var[1]:.4f} z: {mean[2]:.4f},{var[2]:.4f} thx {mean[3]:.2f},{var[3]:.2f} ax {mean[4]:.4f},{var[4]:.4f} thy: {mean[5]:.2f},{var[5]:.2f} ay: {mean[6]:.4f},{var[6]:.4f} thx. {mean[7]:.3f},{var[7]:.3f} thy. {mean[8]:.3f},{var[8]:.3f}", end=" ")
            self.print_to_file(z, mean)

            self.resample()
            self.fluff_particles()
            
            if self.show_current_state_estimation:
                self.plot(self.particles[:, 0], self.particles[:, 1], self.particles[:, 9], 'blue', not self.show_future_prediction)
                self.plot(mean[0], mean[1], 1, 'red', 0)
            self.plot(z[0], z[1], 1, 'black', 0)
            self.show_plot()

            return mean

        def compute_weights(self, z, R):
            distance = np.linalg.norm(self.particles[:, 0:3] - z, axis=1)
            self.particles[:, 9] *= scipy.stats.norm(loc=0, scale=math.sqrt(3*(R**2))).pdf(distance)**2
            self.particles[:, 9] += 1.e-300 # avoid round-off to zero
            # self.accuracy = np.sum(self.particles[:, 9])
            self.particles[:, 9] /= sum(self.particles[:, 9]) # normalize

        def extrapolate_state(self, time):
            future_particles = np.copy(self.particles)
            ax = self.particles[:,4]
            thx = self.particles[:,3]
            alpx = self.particles[:,7]
            # x = x + (axcos(thx + dt*thx.) + sqrt(l^2 + axsin(thx + dt*thx.)^2)) - (axsin(thx) + sqrt(l^2 + axsin(thx)^2))
            future_particles[:,0] = self.particles[:,0] + (ax*np.cos(thx+(time*alpx)) + np.sqrt((self.l**2) + ((ax*np.cos(thx+time*alpx))**2))) - (ax*np.cos(thx) + np.sqrt((self.l**2) + (((ax*np.cos(thx))**2))))
            ay = self.particles[:,6]
            thy = self.particles[:,5]
            alpy = self.particles[:,8]
            # y = y + (aycos(thy + dt*thy.) + sqrt(l^2 + aysin(thy + dt*thy.)^2)) - (aysin(thy) + sqrt(l^2 + aysin(thy)^2))
            future_particles[:,1] = self.particles[:,1] + (ay*np.cos(thy+time*alpy) + np.sqrt((self.l**2) + ((ay*np.cos(thy+time*alpy))**2))) - (ay*np.cos(thy) + np.sqrt((self.l**2) + (((ay*np.cos(thy))**2))))
            future_particles[:, 3] += self.particles[:, 7]*time    #thx += alpx
            future_particles[:, 5] += self.particles[:, 8]*time    #thy += alpy
            return future_particles

        def fluff_particles(self):
            self.particles[:, :9] = np.add(self.particles[:, :9], (np.multiply(np.random.randn(self.N, 9), self.fluff_array))) 

        def constrain_state_variables(self):
            #x,y,z,thx,ax,thy,ay,thx.,thy.
            self.particles[:, 3] %= math.pi*2
            self.particles[:, 5] %= math.pi*2
            l_bound = [-0.5, -0.5, -0.5, -10, -0.1, -10, -0.1, -math.pi*6, -math.pi*6]#[0, 0, 0, 0, 0, 0, 0, 0, 0]
            u_bound = [0.5, 0.5, 0.5, 10, 0.1, 10, 0.1, math.pi*6, math.pi*6]
            self.particles[:, :9] = np.clip(self.particles[:, :9], l_bound, u_bound)

        def resample(self):
            N = len(self.particles[:, 9])
            indexes = np.zeros(N, dtype=int)
            # take int(N*w) copies of each weight
            w = np.asarray(self.particles[:, 9])
            num_copies = (N*w).astype(int)
            k = 0
            for i in range(N):
                for _ in range(num_copies[i]): # make n copies
                    indexes[k] = i
                    k += 1
                    
            # use multinormial resample on the residual to fill up the rest.
            residual = w - num_copies # get fractional part
            residual /= sum(residual) # normalize
            cumulative_sum = np.cumsum(residual)
            cumulative_sum[-1] = 1. # ensures sum is exactly one
            indexes[k:N] = np.searchsorted(cumulative_sum, np.random.random(N-k))

            self.particles[:] = self.particles[indexes]

        def estimate_state(self, particles):
            pos = particles[:, :9]
            mean = np.average(pos, weights=particles[:, 9], axis=0)
            var = np.average((pos - mean)**2, weights=particles[:, 9], axis=0)
            return mean, var

        def plot(self, x_data, y_data, weights, colour, clf):
            if clf: plt.clf()
            plt.scatter(x_data, y_data, label='Particles', color=colour, alpha=(weights/np.max(weights)))

        def show_plot(self):
            plt.xlim(self.window_limits[0], self.window_limits[1])
            plt.ylim(self.window_limits[2], self.window_limits[3])
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Particle Positions')
            plt.show()
            plt.pause(0.01)

        def print_to_file(self, measurement, means):
            self.f = open("Kalman_filter_tracking.csv", "a")
            self.f.write(str(self.count))
            self.f.write(',')
            self.count += 1

            self.f.write(str(measurement[0]))
            self.f.write(',')
            self.f.write(str(measurement[1]))
            self.f.write(',')
            self.f.write(str(measurement[2]))
            self.f.write(',')

            self.f.write(str(means[0]))
            self.f.write(',')
            self.f.write(str(means[1]))
            self.f.write(',')
            self.f.write(str(means[2]))
            self.f.write(',')
            self.f.write(str(means[3]))
            self.f.write(',')
            self.f.write(str(means[4]))
            self.f.write(',')
            self.f.write(str(means[5]))
            self.f.write(',')
            self.f.write(str(means[6]))
            self.f.write(',')
            self.f.write(str(means[7]))
            self.f.write(',')
            self.f.write(str(means[8]))
            self.f.write('\n')
            self.f.close()



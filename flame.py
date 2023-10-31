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
    ### INITIALISATION ###
    # Starting up the global variables
    def __init__(self, fruit = True):
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

    # Connect to the UR5 robot
    def connect(self) -> None:
        robot_host = '192.168.10.20'
        config_filename = 'rtdeCommand.xml'

        self.rtde = rtdeState.RtdeState(robot_host, config_filename)

        self.rtde.initialize()

    # Begin streaming to the UR5
    def begin_stream(self) -> None:
        # Start the stream to output for testing
        self.position_pub = rospy.Publisher("flame/perceived_position", Float64MultiArray, queue_size = 1)
        self.error_pub = rospy.Publisher("flame/position_error", Point, queue_size = 1)

    ### COMMUNICATION ###
    # Appends the current pose of the UR5 to the array of poses
    def receive_tcp_pose(self) -> None:
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

    # Given a certain time stamp it will set the current tcp pose to that of the nearest time match
    def update_pose(self, pose_time) -> None:
        # Set a flag to indicate that a tcp match has been made
        too_early = 1

        # Iterate through all the tcp readings until one is later than the desired time
        for tcp in self.tcps:
            if (tcp != 0):
                if (tcp[1] < pose_time):
                    self.actual_tcp = tcp[0]
                    too_early = 0
                    break

        # If no matching tcp pose was found in memory
        if too_early:
            print("No valid TCP pose was found")

    # Given a point relative to the Realsense convert and send a target to the UR5
    def send_pose(self, position: Point) -> None:
        target = [0, 0, 0, 0, 0, 0]

        # Dealing with the translation first
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
            print(position.z)
            return
        
        # Dealing with the rotation next
        rotation = self.get_rotation()

        # If any of the movement limits are violated do not send the target
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

        # If time greater or equal to the delay time has passed, send the target position
        if time.time() >= self.target_time:
            # Upload the target to the UR5 robotic arm
            for i in range(0, len(target)):
                self.rtde.target_point.__dict__["input_double_register_%i" % i] = target[i]
            self.rtde.con.send(self.rtde.target_point)

            # Set the next time which a message should be sent
            self.target_time = time.time() + self.delay_time

    # Updates all the variables of the code to match that of the actual UR5
    def update_variables(self) -> None:
        state = self.rtde.receive()

        # Read the state variable from the UR5
        self.pick_state = state.output_int_register_0

    # Either enable or disable the servoing variable on the robot
    def send_variables(self, state: int) -> None:
        # Signal to the UR5 that it may execute a servoing command
        self.rtde.servo.input_int_register_0 = state
        self.rtde.con.send(self.rtde.servo)

    # Publishes the target to the predictive motion node
    def publish_position(self, position: Point, position_time: float) -> None:
        # If the position is empty dont try to send it
        if position == []:
            return

        # Publish the global position of the detected fruit for data recording
        pub_point = Float64MultiArray()

        point_data = [0, 0, 0, 0]
        point_data[0] = position.x
        point_data[1] = position.y
        point_data[2] = position.z
        point_data[3] = position_time

        pub_point.data = point_data
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

        print("base x: " + str(round(transformed_point.x, 2)), end="")
        print(" y: " + str(round(transformed_point.y, 2)), end="")
        print(" z: " + str(round(transformed_point.z, 2)))

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
    def nerian_to_base(self, point: Point):
        x = - point.y + 0.6
        y = point.x - 0.025
        z = point.z - 0.65

        return [x, y, z]
    
    ### MISCELLANEOUS ###
    # Given a list of candidate targets relative to the realsense determines the position of the closest target and the global coordinates
    def locate_nearest(self, targets, image_time) -> Point:
        # Updates the pose values of the UR5 and realsense
        self.update_pose(image_time)

        min_error = 1
        
        # Convert all the targets to TCP coordinates and evaluate the best one according to distance from the TCP
        for target in targets:
            # Comparing the distance from the TCP to the fruit
            position = self.realsense_to_tcp(target)
            error = abs(position.x) + abs(position.y) + abs(position.z)

            # Calculating if this is a minimum reading and saving it if it is
            if error < min_error:
                best = position
                min_error = error

        # Calculating the global position of the kiwifruit
        best_position = self.tcp_to_base(best)

        return best_position

    # Returns the rotation value of the UR5 as a point
    def get_rotation(self) -> Point:
        rotation = Point()

        # Approximate the rotation about the y axis to be a sine wave
        rotation.x = 0
        rotation.y = 0 # 0.5 * math.sin(0.1 * (2 * math.pi * time.time()))
        rotation.z = 0

        return rotation

# Defining the camera in hand input and image processing object
class realsense:
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
    def image_to_realsense(self, image_position):
        # Returning the error for the control system to deal with
        position = Point()

        # Doing the calculations to locate the 3D position of a given pixel on the image
        position.z = 0.001 * self.get_depth(image_position)
        position.x = 0.001 * float((image_position[0] - self.x_centre) * position.z * 1000) / self.x_focal
        position.y = 0.001 * float((image_position[1] - self.y_centre) * position.z * 1000) / self.y_focal

        return position
    
    # Given a point on the image the average depth of the neighborhood will be found and offset applied
    def get_depth(self, image_position):
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
        # Converting the received image and extracting the fruit from it
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        fruits = self.extract_fruit(image)

        # Converting the image position of the fruit to a spacial position
        fruit_locations = []

        if fruits is not None:
            for fruit in fruits:
                fruit_locations.append(self.image_to_realsense(fruit))
                cv.circle(image, (fruit[0], fruit[1]), 3, (255, 0, 255), -1)

        cv.imshow("window", image)
        cv.waitKey(1)

        return fruit_locations
  
# Defining the global camera input and output functions
class nerian:
    ### INITIALISATION ###
    # Initialising all the variables
    def __init__(self, fruit):
        # Initialising the image processing values
        self.bridge = CvBridge()

        if fruit:
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

# Defining a object that takes a stream of timestamped positions and predicts the next prediction
class crystal_ball:
    # Initialising the crystal ball
    def __init__(self):
        # Declaring the lookahead time and predition method
        prediction_method = 4
        lookahead = 0.25

        # Declaring the prediction algorithm object
        # 0- Velocity Average, 1- Velocity Smudge, 2- Acceleration Smudge, 3- Acceleration Smudge - Tuned, 4- No Prediction
        self.fruit_predict = self.predict(prediction_method, lookahead)

        # Declaring an evaluation as well as a simulation
        self.fruit_evaluate = self.evaluate()
        self.fruit_simulate = self.simulate(lookahead)

        # Starting all the respective rostopic streams
        self.fruit_evaluate.start_stream()

    # Starting up the ros nodes for receiving and sending positions
    def start_stream(self):
        self.point_pub = rospy.Publisher("flame/predicted_position", Point, queue_size = 1)
 
    # Callback for recieving fruit positions
    def predict_position(self, data):
        fruit_position = data.data

        self.fruit_predict.predict_position(fruit_position)
        
        predicted_point = Point()

        predicted_point.x = self.fruit_predict.predicted_pos[0]
        predicted_point.y = self.fruit_predict.predicted_pos[1]
        predicted_point.z = self.fruit_predict.predicted_pos[2]

        self.point_pub.publish(predicted_point)

        # Calculate prediction error
        self.fruit_evaluate.compare(fruit_position, self.fruit_predict.predicted_pos)

    # Continuously runs a simulation with a specific motion until the q key is pressed
    def start_simulation(self):
        while True:
            future_pos, fruit_position = self.fruit_simulate.stream_simulation()

            # Run prediction model on simulation
            self.fruit_predict.predict_position(fruit_position)
            print("predicted: ", self.fruit_predict.predicted_pos)

            # Evaluate the prediction error
            self.fruit_evaluate.compare(fruit_position, self.fruit_predict.predicted_pos)

            # Show visual of fruit
            self.fruit_evaluate.display(future_pos, self.fruit_predict.predicted_pos)

            # Run simulation at desired speed, exit on 'q'
            if (cv.waitKey(int((1000/self.fruit_simulate.simulation_sampling_frequency)/self.fruit_simulate.playback_speed)) & 0xFF == ord('q')):
                break

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
        def predict_position(self, pos):
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
            self.predict_pub = rospy.Publisher("flame/prediction_error", Float64MultiArray, queue_size = 1)

        # Compares streams of position and predicted_position
        def compare(self, position, predicted_position):

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
            pub_prediction = Float64MultiArray()

            pub_prediction_data = [0, 0, 0, 0]
            pub_prediction_data[0] = float(error[0])
            pub_prediction_data[1] = float(error[1])
            pub_prediction_data[2] = float(error[2])
            pub_prediction_data[3] = float(error[3])

            pub_prediction.data = pub_prediction_data
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
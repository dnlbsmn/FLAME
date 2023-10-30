import math
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

# Import the ROS libraries
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point

lookahead = 0.25

expected_samples_per_second = 60

#0- vel average
#1- vel bayes
#2- acc bayes
#3- making newton sad
#4- no prediction
PREDICT_METHOD = 3
ACC = 1

if ACC == 0:
    GAUSSIAN_SIZE = 101
    GAUSSIAN_DIST = 0.35
    BAYES_STORED_VALUES = 30
    BAYES_ARRAY_SIZE = 301
    DIMINISHING_FACTOR = 0.65
    SLIDING_WINDOW_WIDTH_VEL = 10
    SLIDING_WINDOW_WIDTH_ACC = 12
    VELOCITY_GAIN = 0.8 #for method 3
    ACCELERATION_GAIN = 0

else:
    GAUSSIAN_SIZE = 101
    GAUSSIAN_DIST = 0.35
    BAYES_STORED_VALUES = 30
    BAYES_ARRAY_SIZE = 301
    DIMINISHING_FACTOR = 0.65
    SLIDING_WINDOW_WIDTH_VEL = 8
    SLIDING_WINDOW_WIDTH_ACC = 12
    VELOCITY_GAIN = 0.4 #for method 3
    ACCELERATION_GAIN = 0.65



DISPLAY_MOVEMENT_GAIN = 5000

class predict:
    def __init__(self):
        self.fruit_pos = np.zeros((expected_samples_per_second, 4))
        self.fruit_vel = np.zeros((expected_samples_per_second-1, 4))
        self.fruit_acc = np.zeros((expected_samples_per_second-2, 4))
        self.fruit_jrk = np.zeros((expected_samples_per_second-3, 4))
        self.predicted_pos = [0, 0, 0, 0]
        self.gauss_size = GAUSSIAN_SIZE
        self.gauss = self.gaussuian_filter(self.gauss_size, GAUSSIAN_DIST, 0)

    def predict_position(self, pos, lookahead_time, method):
        # push onto shift register
        self.push_fruit_pos_sr(pos)

        # print("SR:", self.fruit_pos[0:5])
        
        #Velocity lookahead
        if (method == 0):
            self.get_derrivatives(1, 1)
            vel = self.recent_average(self.fruit_vel, 3)
            self.velocity_predict(vel, lookahead_time)

        #Bayes vel
        elif (method == 1):
            self.get_derrivatives(1, SLIDING_WINDOW_WIDTH_VEL)
            # print("vel: ", self.fruit_vel[0:5])
            # print("acc: ", self.fruit_acc[0:5])
            self.bayes_predict(lookahead_time, BAYES_STORED_VALUES, BAYES_ARRAY_SIZE, PREDICT_METHOD)

        #Bayes acc
        elif (method == 2):
            self.get_derrivatives(2, SLIDING_WINDOW_WIDTH_VEL, SLIDING_WINDOW_WIDTH_ACC)
            # print("vel: ", self.fruit_vel[0:5])
            # print("acc: ", self.fruit_acc[0:5])
            self.bayes_predict(lookahead_time, BAYES_STORED_VALUES, BAYES_ARRAY_SIZE, PREDICT_METHOD)

        #Making newton sad
        elif (method == 3):
            self.get_derrivatives(2, SLIDING_WINDOW_WIDTH_VEL, SLIDING_WINDOW_WIDTH_ACC)
            # print("vel: ", self.fruit_vel[0:5])
            # print("acc: ", self.fruit_acc[0:5])
            self.bayes_predict(lookahead_time, BAYES_STORED_VALUES, BAYES_ARRAY_SIZE, PREDICT_METHOD, VELOCITY_GAIN, ACCELERATION_GAIN)

        #no pred
        elif (method == 4):
            self.predicted_pos = pos

    def push_fruit_pos_sr(self, pos):
        self.fruit_pos = np.roll(self.fruit_pos, 1, 0)
        self.fruit_pos[0] = pos

    def get_derrivatives(self, n, wv, wa=1):
        if (n>0):
            if wv == 1:
                self.fruit_vel = self.find_derrivative(self.fruit_pos)
            if wv > 1:
                self.fruit_vel = self.find_derrivative_sliding_window(self.fruit_pos, wv)
        if (n>1):
            if wa == 1:
                self.fruit_acc = self.find_derrivative(self.fruit_vel)
            if wa > 1:
                self.fruit_acc = self.find_derrivative_sliding_window(self.fruit_vel, wa)
        if (n>2):
            self.fruit_jrk = self.find_derrivative(self.fruit_acc)
        
    def find_derrivative(self, arr):
        derr_arr = np.zeros((len(arr)-1, 4))
        next_el = arr[0]
        i = 0
        for el in arr[1:]:
            dt = next_el[3]-el[3]
            
            if (dt==0):
                dt = 1/expected_samples_per_second

            derr_arr[i][0] = (next_el[0] - el[0])/dt
            derr_arr[i][1] = (next_el[1] - el[1])/dt
            derr_arr[i][2] = (next_el[2] - el[2])/dt
            derr_arr[i][3] = el[3]
            next_el = el
            i += 1
        return derr_arr

    def find_derrivative_sliding_window(self, arr, w):
        derr_arr = np.zeros((len(arr)-w, 4))
        for i in range(len(arr[1:len(arr)-w])):
            dt = 0
            derr_arr[i] = [0, 0, 0, 0]
            for j in range(w):
                dt += (arr[i+j][3] - arr[i+j+1][3])/w
            if (dt==0):
                dt = 1/expected_samples_per_second
            for k in range(w):
                derr_arr[i][0] += (arr[i+k][0] - arr[i+k+1][0])/(dt*w)
                derr_arr[i][1] += (arr[i+k][1] - arr[i+k+1][1])/(dt*w)
                derr_arr[i][2] += (arr[i+k][2] - arr[i+k+1][2])/(dt*w)
            derr_arr[i][3] = arr[int(i+(w/2))][3]
        return derr_arr  
    
    def bayes_predict(self, lookahead, n, sz, method, velocity_gain=1, acceleration_gain=1):
        if method == 1:
            pred = self.interpolate_vel(lookahead, n)
        elif (method == 2) or (method == 3):
            pred = self.interpolate_acc(lookahead, n, velocity_gain, acceleration_gain)
        spread = np.zeros((sz, sz))
        c = int((sz-self.gauss.shape[0])/2)
        spread[c:c+self.gauss.shape[0], c:c+self.gauss.shape[1]] = self.gauss
        x_init = pred[0][0]
        y_init = pred[0][1]
        for p in range(len(pred)-1):
            x_offs = int((pred[p+1][0] - x_init)*2000)
            y_offs = int((pred[p+1][1] - y_init)*2000)
            if (abs(x_offs) < (sz-self.gauss_size)/2) & (abs(y_offs) < (sz-self.gauss_size)/2):
                spread[c+x_offs:c+x_offs+self.gauss.shape[0], c+y_offs:c+y_offs+self.gauss.shape[1]] += (DIMINISHING_FACTOR**(p+1))*self.gauss
            else:
                print('omitted: ', p)

        predicted_indx = np.where(spread == spread.max())
        predicted_position = [x_init + ((predicted_indx[0][0]-((sz-1)/2))/2000), 
                              y_init + ((predicted_indx[1][0]-((sz-1)/2))/2000), 
                              self.fruit_pos[0][2], self.fruit_pos[0][3] + lookahead]
        self.predicted_pos = predicted_position
        spread = cv.circle(spread, (int(predicted_indx[1][0]), int(predicted_indx[0][0])), 3, (0, 0, 0), 1)
        cv.imshow('spread', spread)

    def interpolate_acc(self, lookahead_time, n, p, q):
        pred = np.zeros((n, 4))
        lookahead_ts = self.fruit_pos[0][3] + lookahead_time
        for i in range(n):
            dt = lookahead_ts - self.fruit_pos[i][3]
            pred[i][0] = self.fruit_pos[0][0] + (p*self.fruit_vel[i][0]*dt) + 0.5*(q*self.fruit_acc[i][0]*(dt**2))
            pred[i][1] = self.fruit_pos[0][1] + (p*self.fruit_vel[i][1]*dt) + 0.5*(q*self.fruit_acc[i][1]*(dt**2))
            pred[i][2] = self.fruit_pos[0][2] + (self.fruit_vel[i][2]*dt) + 0.5*(self.fruit_acc[i][2]*(dt**2))
        return pred
    
    def interpolate_vel(self, lookahead_time, n):
        pred = np.zeros((n, 4))
        lookahead_ts = self.fruit_pos[0][3] + lookahead_time
        for i in range(n):
            dt = lookahead_ts - (self.fruit_pos[i][3])
            pred[i][0] = self.fruit_pos[0][0] + (self.fruit_vel[i][0]*dt)
            pred[i][1] = self.fruit_pos[0][1] + (self.fruit_vel[i][1]*dt)
            pred[i][2] = self.fruit_pos[0][2] + (self.fruit_vel[i][2]*dt)
        return pred

    def velocity_predict(self, vel, lookahead_time):
        predicted_pos = [0,0,0,0]
        pos = self.fruit_pos[0]
        predicted_pos[0] = pos[0] + vel[0]*lookahead_time
        predicted_pos[1] = pos[1] + vel[1]*lookahead_time
        predicted_pos[2] = pos[2] + vel[2]*lookahead_time
        predicted_pos[3] = pos[3] + lookahead_time
        self.predicted_pos = predicted_pos

    def gaussuian_filter(self, kernel_size, sigma=0.5, muu=0):
        x, y = np.meshgrid(np.linspace(-1, 1, kernel_size), np.linspace(-1, 1, kernel_size))
        dst = np.sqrt(x**2+y**2)
        gauss = np.exp(-((dst-muu)**2 / (2.0 * sigma**2)))
        return gauss

    def recent_average(self, arr, n):
        vec = [0, 0, 0, 0] #xyzt
        #next_el = arr[0]
        for i in range(n):
            vec[0] += (arr[i][0])/n
            vec[1] += (arr[i][1])/n
            vec[2] += (arr[i][2])/n
        vec[3] = arr[0][3]
        return vec

class evaluate:
    def __init__(self):
        self.empty = np.zeros((1000, 1000, 3))
        self.predicted_positions = [[0, 0, 0, 0]]
        self.error_stats = []
        self.flag_c = 0
        self.first_flag = 0
        self.first_position = [0, 0, 0, 0]

    def compare(self, position, predicted_position, UI):
        predicted_position = self.check_predicted_position(position, predicted_position)
        self.calculate_error(position, predicted_position, UI)
        self.display(position, predicted_position, UI)

    def display(self, position, predicted_position, UI):
        if (UI != 0):
            movement_gain = DISPLAY_MOVEMENT_GAIN

            self.flag_c += 1
            if (self.flag_c > 30) & (self.first_flag == 0):
                self.first_position = position
                self.first_flag = 1

            im_below = np.zeros((500, 500, 3))
            im_below = cv.circle(im_below, (250+int(self.first_position[0] + movement_gain*position[0] ), 250+int(self.first_position[1] + movement_gain*position[1])), 5, (255, 255, 255), -1)
            im_below = cv.circle(im_below, (250+int(self.first_position[0] + movement_gain*predicted_position[0]), 250+int(self.first_position[1] + movement_gain*predicted_position[1])), 5, (0, 0, 255), -1)
            # cv.imshow('below', im_below)

            im_side = np.zeros((500, 500, 3))
            im_side = cv.circle(im_side, (250+int(self.first_position[0] + movement_gain*position[0]), 250+int(self.first_position[2] + movement_gain*position[2])), 5, (255, 255, 255), -1)
            im_side = cv.circle(im_side, (250+int(self.first_position[0] + movement_gain*predicted_position[0]), 250+int(self.first_position[2] + movement_gain*predicted_position[2])), 5, (0, 0, 255), -1)
            # cv.imshow('side', im_side)

    def check_predicted_position(self, pos, predicted_pos):
        for i in range(len(predicted_pos)):
            if (np.isnan(predicted_pos[i]) == 1):
                predicted_pos[i] = pos[i]
                print('Warning: NaN detected')
        return predicted_pos
    
    def calculate_error(self, pos, predicted_pos, UI):
        del_idx = []
        error = [0, 0, 0, 0]
        self.predicted_positions = np.concatenate((self.predicted_positions, [predicted_pos]))

        time_diff = 1
        nearest_idx = 0
        for p in range(len(self.predicted_positions)):

            if (abs(pos[3] - self.predicted_positions[p][3]) < time_diff):
                time_diff = abs(pos[3] - self.predicted_positions[p][3])
                nearest_idx = p

            if (pos[3] > self.predicted_positions[p][3]):
                del_idx.append(p)
                
        error[0] = pos[0] - self.predicted_positions[nearest_idx][0]
        error[1] = pos[1] - self.predicted_positions[nearest_idx][1]
        error[2] = pos[2] - self.predicted_positions[nearest_idx][2]
        error[3] = pos[3] - self.predicted_positions[nearest_idx][3]
        abs_error = math.sqrt((error[0]**2) + (error[1]**2) + (error[2]**2))

        self.predicted_positions = np.delete(self.predicted_positions, del_idx, 0)

        if (UI == 2):
            print(abs_error)

        pub_prediction = Float64MultiArray()

        pub_prediction_data = [0, 0, 0, 0]
        pub_prediction_data[0] = float(error[0])
        pub_prediction_data[1] = float(error[1])
        pub_prediction_data[2] = float(error[2])
        pub_prediction_data[3] = float(error[3])

        pub_prediction.data = pub_prediction_data
        predict_pub.publish(pub_prediction)

def callback(data):
    fruit_position = data.data

    hobbs.predict_position(fruit_position, lookahead_time=lookahead, method=PREDICT_METHOD)
    
    predicted_point = Point()

    predicted_point.x = hobbs.predicted_pos[0]
    predicted_point.y = hobbs.predicted_pos[1]
    predicted_point.z = hobbs.predicted_pos[2]

    point_pub.publish(predicted_point)

    mother.compare(fruit_position, hobbs.predicted_pos, UI=1)

    cv.waitKey(1)

hobbs = predict()
mother = evaluate()

rospy.init_node('tracer', anonymous=True)
point_sub = rospy.Subscriber("/flame/perceived_position", Float64MultiArray, callback)
point_pub = rospy.Publisher("flame/predicted_position", Point, queue_size = 1)
predict_pub = rospy.Publisher("flame/prediction_error", Float64MultiArray, queue_size = 1)

rospy.spin()
############################
# count = 0
#while(1):
    # count += 0.1
    # x = float(input("input x: "))
    # y = float(input("input y: "))
    # z = float(input("input z: "))
    # t = count

    #print("time: ", t)
# for x in range(100):
#     hobbs.predict_position([x/1000, 0, 0, x/10], 0.1, 2)
#     mother.compare([x/1000, 0, 0, x/10], hobbs.predicted_pos, UI=1)
#     print("pos: ", x/1000, " t: ", x/10)
#     print(hobbs.predicted_pos)

######################################SIM
# stem_length = 0.04
# theta_0 = 0
# phi_0 = math.pi
# A_x = 0.02
# f_th = 1
# A_y = 0
# f_ph = 0
# simulation_sampling_frequency = 50
# playback_speed = 1
# class fruit_sim:
#     def __init__(self):
#         self.r = stem_length
#         self.A_th = math.asin(A_x/self.r)
#         self.A_ph = math.asin(A_y/self.r)
#         self.theta = theta_0
#         self.phi = phi_0
#         self.sphr_pos = [self.r, self.theta, self.phi]
#         self.cart_pos = [0,0,0]
#         self.fruit_p = [0,0,0,0]
#         self.phase_th = 0
#         self.phase_ph = 0
#         self.time_stamp = 0
#         self.fruit_SR = np.zeros((int((simulation_sampling_frequency*lookahead)+1), 4))
    
#     def inc_fruit_position(self):
#         self.phase_th += f_th*2*math.pi/simulation_sampling_frequency
#         self.theta = theta_0 + self.A_th*math.sin(self.phase_th)

#         if (self.phase_th >= 2*math.pi):
#             self.phase_th -= 2*math.pi

#         self.phase_ph += f_ph*2*math.pi/simulation_sampling_frequency
#         self.phi = phi_0 + self.A_ph*math.sin(self.phase_ph)

#         if (self.phase_ph >= 2*math.pi):
#             self.phase_ph -= 2*math.pi

#         self.time_stamp += 1/simulation_sampling_frequency
#         self.sphr_pos = [self.r, self.theta, self.phi]
#         self.sphr_to_cart()

#     def sphr_to_cart(self):
#         self.cart_pos[0] = self.sphr_pos[0] * math.sin(self.sphr_pos[1]) * math.cos(self.sphr_pos[2])
#         self.cart_pos[1] = self.sphr_pos[0] * math.sin(self.sphr_pos[1]) * math.sin(self.sphr_pos[2])
#         self.cart_pos[2] = self.sphr_pos[0] * math.cos(self.sphr_pos[1])
    
#     def gen_fruit_pos(self):
#         self.fruit_p = np.concatenate((self.cart_pos, [self.time_stamp]))

#     def send_delayed(self):
#         self.fruit_SR = np.roll(self.fruit_SR, 1, 0)
#         self.fruit_SR[0] = self.fruit_p
#         #print(self.fruit_SR)
#         return self.fruit_SR[0], self.fruit_SR[-1]

#     def printy(self):
#         print(self.time_stamp, self.cart_pos)

#     def stream_simulation(self, UI):
#         self.inc_fruit_position()
#         self.gen_fruit_pos()
#         if (UI): 
#             self.printy()
#         future_pos, sim = self.send_delayed()
#         return future_pos, sim

# calvin = fruit_sim()

# while(1):
#     future_pos, fruit_position = calvin.stream_simulation(UI=0)
#     hobbs.predict_position(fruit_position, lookahead_time=lookahead, method=PREDICT_METHOD)
#     print("predicted: ", hobbs.predicted_pos)
#     mother.compare(future_pos, hobbs.predicted_pos, UI=1)
#     if (cv.waitKey(int((1000/simulation_sampling_frequency)/playback_speed)) & 0xFF == ord('q')):
#         break
#######################################

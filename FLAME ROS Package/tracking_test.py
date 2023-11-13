# Importing general python libraries
from turtle import position
import rospy
import time
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray

ROUNDING = 8
TEST_DURATION = 30

global first_reading_flag

def write_data():
    global prediction_error, perceived_position, position_error

    f.write(str(round((time.time() - start_time), 3)))
    f.write(',')
    f.write(str(round((position_error.x), ROUNDING)))
    f.write(',')
    f.write(str(round((position_error.y), ROUNDING)))
    f.write(',')
    f.write(str(round((position_error.z), ROUNDING)))
    f.write(',')
    f.write(str(round((prediction_error[0]), ROUNDING)))
    f.write(',')
    f.write(str(round((prediction_error[1]), ROUNDING)))
    f.write(',')
    f.write(str(round((prediction_error[2]), ROUNDING)))
    f.write(',')
    f.write(str(round((perceived_position[0]), ROUNDING)))
    f.write(',')
    f.write(str(round((perceived_position[1]), ROUNDING)))
    f.write(',')
    f.write(str(round((perceived_position[2]), ROUNDING)))
    f.write('\n')


# Defining all the functions that write the received messages to global variables
def receive_prediction_error(prediction_array):
    global prediction_error
    prediction_error = prediction_array.data

def receive_perceived_position(array):
    global perceived_position
    perceived_position = array.data

def receive_position_error(error):
    global position_error
    position_error = error

def end_test():
    f.close()
    print("Test ended")

def start_test():
    global start_time
    start_time = time.time()

    global first_reading_flag
    first_reading_flag = 0


    rospy.init_node('test_node', anonymous=True)
    rospy.Subscriber("flame/position_error", Point, receive_position_error)
    rospy.Subscriber("flame/prediction_error", Float64MultiArray, receive_prediction_error)
    rospy.Subscriber("flame/perceived_position", Float64MultiArray, receive_perceived_position)
    rospy.on_shutdown(end_test)

    loop_time = 0

    # Write to the output file once every 16 milliseconds
    while (time.time() - start_time) <= TEST_DURATION:
        if (time.time() - loop_time) > 0.016:
            write_data()
            loop_time = time.time()
    
    exit()

if __name__ == "__main__":
    file_name = input("Enter file name: ")

    f = open("Benchmark_data/" + file_name + ".csv", "w")
    # f.write("In order of occurence position error (x y z) prediction_error (x y z) perceived_position (x y z)\n")
    f.write("time,position_error x,position_error y,position_error z,prediction_error x,prediction_error y,prediction_error z,perceived_position x,perceived_position y,perceived_position z\n")

    print("Data is being recorded")

    start_test()
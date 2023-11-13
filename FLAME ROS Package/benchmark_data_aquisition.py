# Importing general python libraries
import rospy
import time
from geometry_msgs.msg import PointStamped

ROUNDING = 8
TEST_DURATION = 30

global first_reading_flag

def receive_position(pos):
    global first_reading_flag
    global t_prev, x_prev, y_prev, z_prev
    global start_time

    if first_reading_flag == 0:
        t_prev = pos.header.stamp.secs + (pos.header.stamp.nsecs/10**9)
        x_prev = pos.point.x
        y_prev = pos.point.y
        z_prev = pos.point.z
        first_reading_flag = 1
        start_t = time.time()

    else:
        timestamp = pos.header.stamp.secs + (pos.header.stamp.nsecs/10**9) - start_time
        dt = timestamp - t_prev
        f.write(str(round((timestamp), 5)))
        f.write(',')
        f.write(str(round((pos.point.x), ROUNDING)))
        f.write(',')
        f.write(str(round((pos.point.y), ROUNDING)))
        f.write(',')
        f.write(str(round((pos.point.z), ROUNDING)))
        f.write(',')
        f.write(str(round((pos.point.x-x_prev)/dt, ROUNDING)))
        f.write(',')
        f.write(str(round((pos.point.y-y_prev)/dt, ROUNDING)))
        f.write(',')
        f.write(str(round((pos.point.z-z_prev)/dt, ROUNDING)))
        f.write(',')
        f.write(str(round((pos.point.x-x_prev), ROUNDING)))
        f.write(',')
        f.write(str(round((pos.point.y-y_prev), ROUNDING)))
        f.write(',')
        f.write(str(round((pos.point.z-z_prev), ROUNDING)))
        f.write('\n')

        #t_prev = pos.header.stamp.secs + (pos.header.stamp.nsecs/10**9)
        x_prev = pos.point.x
        y_prev = pos.point.y
        z_prev = pos.point.z
        t_prev = pos.header.stamp.secs + (pos.header.stamp.nsecs/10**9) - start_time

def end_test():
    f.close()
    print("Test ended")

def start_test():
    global start_time
    start_time = time.time()

    global first_reading_flag
    first_reading_flag = 0


    rospy.init_node('test_node', anonymous=True)
    rospy.Subscriber("/flame/perceived_position", PointStamped, receive_position)
    rospy.on_shutdown(end_test)

    # Write to the output file once every 16 milliseconds
    while (time.time() - start_time) <= TEST_DURATION:
        pass
    
    exit()

if __name__ == "__main__":
    file_name = input("Enter file name: ")

    f = open("Benchmark_data/" + file_name + ".csv", "w")
    # f.write("In order of occurence position error (x y z) prediction_error (x y z) perceived_position (x y z)\n")
    f.write("time,position x,position y,position z, velocity x, velocity y, velocity z, dx, dy, dz\n")

    print("Data is being recorded")

    start_test()
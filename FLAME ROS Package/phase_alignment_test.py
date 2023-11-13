#!/usr/bin/env python3

### ================================ ###
# INITIALISATION

# Importing all the basic Python libraries
from flame import ur_five
import time
from geometry_msgs.msg import Point
import math

### ================================ ###
# THE MAIN WORKFLOW OF THE SYSTEM

def write_tcps(timestamp, actual, target):
    # Updating the parameters of the arm

    # Writing out to the file
    f.write(str(round(timestamp, 3)))
    f.write(',')
    f.write(str(round((actual[0]), 4)))
    f.write(',')
    f.write(str(round((actual[1]), 4)))
    f.write(',')
    f.write(str(round((actual[2]), 4)))
    f.write(',')
    f.write(str(round((target.x), 4)))
    f.write(',')
    f.write(str(round((target.y), 4)))
    f.write(',')
    f.write(str(round((target.z), 4)))
    f.write('\n')

if __name__ == '__main__':
    # Defining the motion constants of the test
    amplitudes = [50] * 20 + [30] * 20 + [15] * 20 + [10] * 20
    frequencies = ([2.7] * 5 + [2.2] * 5 + [1.4] * 5 + [0.5] * 5) * 4

    # Initialising the UR5
    jerry = ur_five()
    jerry.connect()
    jerry.begin_stream()
    jerry.enable_servoing()

    # Opening up the results file and writing the phase shift to it
    results = open("phase_shift_results.csv", "a")

    # Iterating through all the tests in one go
    for number in range(60):
        # Setting the motion characteristics
        frequency = frequencies[number]
        amplitude = float(amplitudes[number]) / 1000

        # Starting the timer and opening the output file
        f = open("phase_test_" + str(number) + ".csv", "w")
        f.write("time,actual x,actual y,actual z,target x,target y,target z\n")

        # Setting the maximum value variables
        max_target = 0
        max_actual = 0

        max_target_time = 0
        max_actual_time = 0

        ### The initial wait time to pad the data
        test_time = time.time()
        start_time = time.time()
        while time.time() < start_time + 0.5:
            target = Point()

            target.x = 0.5
            target.y = 0
            target.z = 0.5

            timestamp = time.time() - test_time
            jerry.send_target(target)

            # Updating the positions of the UR5 and writing them to a file
            jerry.update_state()
            actual = jerry.actual_tcp
            write_tcps(timestamp, actual, target)

        ### The oscillation generation segment
        start_time = time.time()
        while time.time() < start_time + 1 / frequency:
            # Send the next servoing command
            target = Point()

            target.x = 0.5 + amplitude * math.sin(frequency * 2 * math.pi * (time.time() - start_time))
            target.y = 0
            target.z = 0.5

            timestamp = time.time() - test_time
            jerry.send_target(target)

            # Updating the positions of the UR5 and writing them to a file
            jerry.update_state()
            actual = jerry.actual_tcp

            write_tcps(timestamp, actual, target)

            # Checking if the actual tcp or target tcp is at the maximum
            if actual[0] > max_actual:
                max_actual = actual[0]
                max_actual_time = timestamp

            if target.x > max_target:
                max_target = target.x
                max_target_time = timestamp

        # The post test data padding
        start_time = time.time()
        while time.time() < start_time + 0.5:
            target = Point()

            target.x = 0.5
            target.y = 0
            target.z = 0.5

            timestamp = time.time() - test_time
            jerry.send_target(target)

            # Updating the positions of the UR5 and writing them to a file
            jerry.update_state()
            actual = jerry.actual_tcp
            write_tcps(timestamp, actual, target)

        f.close()
        print("Test complete")

        results.write(str(number))
        results.write(',')
        results.write(str(round(amplitude, 4)))
        results.write(',')
        results.write(str(round(frequency, 4)))
        results.write(',')
        # Writing the time between the motion peaks
        results.write(str(round(max_target_time, 4)))
        results.write(',')
        results.write(str(round(max_actual_time, 4)))
        results.write(',')
        results.write(str(round(max_actual_time - max_target_time, 4)))
        results.write(',')
        # Writing the percentage of the target amplitude that was achieved
        results.write(str(round(max_target, 4)))
        results.write(',')
        results.write(str(round(max_actual, 4)))
        results.write('\n')

    results.close()
    print("All tests complete")
#!/usr/bin/env python3


# CONCLUSION
# The delay for rostopics to transfer data is less than a millisecond for 
# a 500 character string, so transferring data that is not an image won't 
# drastically increase delay

import rospy
import time
from std_msgs.msg import String

def callback(data):
    global sum, num

    delay = round((time.time() - start_time) * 1000, 2)

    sum += delay
    num += 1

    print("Average delay: " + str(round(sum / num, 2)) + " ms")

rospy.init_node("tester", anonymous=True)

pub = rospy.Publisher("/flame/test", String, queue_size = 1)
sub = rospy.Subscriber("/flame/test", String, callback, queue_size = 1)

time.sleep(0.1)

sum = 0
num = 0

for i in range(500):
    start_time = time.time()
    pub.publish("Hello" * 100)
    time.sleep(0.01)

time.sleep(0.1)

print("Done")
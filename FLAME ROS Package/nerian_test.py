# Importing the libraries

import rospy
from flame import nerian
from flame import ur_five

from sensor_msgs.msg import Image

def callback(image):
    fruit = greg.process_image(image)

    if fruit != []:
        global_fruit = jerry.nerian_to_position(fruit[0])

# Initialising the listener node
greg = nerian()
rospy.init_node('listener', anonymous=True)
greg.load_parameters()

jerry = ur_five()

image_sub = rospy.Subscriber("/nerian_stereo/left_image", Image, callback, queue_size = 1)

rospy.spin()
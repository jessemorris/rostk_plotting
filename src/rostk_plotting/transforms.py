from rospy_message_converter import message_converter

import rospy
import ros_numpy

def ros_msg_to_dict(data):
    return message_converter.convert_ros_message_to_dictionary(data)


def image_msg_to_np(sensor_image):
    return ros_numpy.numpify(sensor_image)

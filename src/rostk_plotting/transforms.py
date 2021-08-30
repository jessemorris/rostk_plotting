from rospy_message_converter import message_converter

import rospy
import ros_numpy
from sensor_msgs.point_cloud2 import read_points
import numpy as np

def ros_msg_to_dict(data):
    return message_converter.convert_ros_message_to_dictionary(data)


def image_msg_to_np(sensor_image):
    return ros_numpy.numpify(sensor_image)

def point_cloud_to_ndarray(point_cloud):
    points_list = []
    for p in read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
        points_list.append(p)

    return np.array(points_list)



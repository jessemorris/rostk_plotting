import rospy
import numpy as np
from geometry_msgs.msg import Point

if __name__ == "__main__":
    rospy.init_node("point_publisher")

    pub = rospy.Publisher("point_tests", Point, queue_size=1)
    pub1 = rospy.Publisher("point_tests1", Point, queue_size=1)
    point = Point()
    point.x = 0
    point.y = 0
    point.z = 0

    while not rospy.is_shutdown():
        time = rospy.get_rostime().secs
        point.x = 2 * np.sin(0.5 * time)
        point.y = 2 * np.cos(0.5 * time)
        point.z = 2 * np.tan(0.5 * time)
        pub.publish(point)
        pub1.publish(point)
        rospy.sleep(1)
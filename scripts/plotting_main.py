#!/usr/bin/env python
from threading import Thread, Condition
import message_filters
import roslib
import rospy
import cv2
import signal
from datetime import datetime


from geometry_msgs.msg import Twist
from rostk_plotting.dynamic_import import DynamicImport
from rostk_plotting.plotting_callbacks import PlottingCallbacks
from rostk_plotting.plotting_manager import PlottingManager, key_event
import sys, select, termios, tty, os
import rospkg

rospack = rospkg.RosPack()
package_path = rospack.get_path("rostk_plotting")

def get_key(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def signal_handler(signal, frame):
    raise KeyboardInterrupt('SIGINT received')


class ScreenShotPlotting(PlottingManager, PlottingCallbacks):

    def __init__(self, topic_list, queue_size, slop_time):
        PlottingCallbacks.__init__(self)
        PlottingManager.__init__(self, topic_list, queue_size, slop_time, self)

        self.info_flag = True

    def parse_command(self, command_string):
        pass

    @key_event("info_flag")
    def camera_info_callback(self, data):
        print("In better camera info")


    def image_callback(self, data):
        print("in better image callback")



if __name__=="__main__":

    rospy.init_node("rostk_plotting", disable_signals=True)
    signal.signal(signal.SIGINT, signal_handler)

    snapshot_topic_list = rospy.get_param('/ros_toolkit/plotting/snapshot_topics')
    queue_size = rospy.get_param('/ros_toolkit/plotting/queue_size')
    slop_time = rospy.get_param('/ros_toolkit/plotting/slop_time')

    record_path = package_path + "/records"
    manager = ScreenShotPlotting(snapshot_topic_list, int(queue_size), int(slop_time))

    
    
    while not rospy.is_shutdown():   
        #we are in python2.7! 
        try:  
            input_command = str(raw_input())
        except KeyboardInterrupt:
            rospy.signal_shutdown("CTR-C recieved")
            break

        

    
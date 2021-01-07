#!/usr/bin/env python
from threading import Thread, Condition
import message_filters
import roslib
import rospy
import cv2
import signal
from datetime import datetime
import ros_numpy


from rostk_plotting.dynamic_import import DynamicImport
from rostk_plotting.transforms import *
from rostk_plotting.plotting_callbacks import PlottingCallbacks
from rostk_plotting.plotting_manager import PlottingManager, attribute_event
import sys, select, termios, tty, os
import rospkg




def signal_handler(signal, frame):
    raise KeyboardInterrupt('SIGINT received')


class ScreenShotPlotting(PlottingManager, PlottingCallbacks):

    def __init__(self, topic_list, queue_size, slop_time):
        PlottingCallbacks.__init__(self)
        PlottingManager.__init__(self, topic_list, queue_size, slop_time, self)

        self.screen_cap_flag = False
        self.image_id = 0

    def post_callback(self):
        self.screen_cap_flag = False

    def on_shutdown(self):
        print("Goodbye")

    def parse_command(self, command_string):
        if command_string == "sc":
            self.screen_cap_flag = True

    @attribute_event("screen_cap_flag")
    def camera_info_callback(self, data):
        pass

    @attribute_event("screen_cap_flag")
    def image_callback(self, data):
        input_image = image_msg_to_np(data)
        cv2.imwrite(ScreenShotPlotting.get_current_record_folder() + str(self.image_id) + ".png", input_image)
        self.image_id+=1



if __name__=="__main__":

    rospy.init_node("rostk_plotting", disable_signals=True)
    signal.signal(signal.SIGINT, signal_handler)

    snapshot_topic_list = rospy.get_param('/ros_toolkit/plotting/snapshot_topics')
    queue_size = rospy.get_param('/ros_toolkit/plotting/queue_size')
    slop_time = rospy.get_param('/ros_toolkit/plotting/slop_time')

    manager = ScreenShotPlotting(snapshot_topic_list, int(queue_size), int(slop_time))

    
    
    while not rospy.is_shutdown():   
        #we are in python2.7! 
        try:  
            input_command = str(raw_input())
            PlottingManager.execute_commands(input_command)
        except KeyboardInterrupt:
            rospy.signal_shutdown("CTR-C recieved")
            
            clear_records = str(raw_input("Would you like to clear records just made: {} [y/n]".format(PlottingManager.get_current_record_folder())))
            if clear_records == "y":
                print("Clearing records")
                PlottingManager.clear_record()

            PlottingManager.finalise()


        

    
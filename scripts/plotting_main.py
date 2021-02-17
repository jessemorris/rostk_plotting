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
from rostk_plotting.screen_shot import ScreenShotPlotting
from rostk_plotting.continuous_record import ContinuousPlotting
from rostk_plotting.graphing import Graphing
import sys, select, termios, tty, os
import rospkg




def signal_handler(signal, frame):
    raise KeyboardInterrupt('SIGINT received')





if __name__=="__main__":

    rospy.init_node("rostk_plotting", disable_signals=True)
    signal.signal(signal.SIGINT, signal_handler)

    snapshot_topic_list = rospy.get_param('/ros_toolkit/plotting/snapshot_topics')
    # video_topic_list = rospy.get_param('/ros_toolkit/plotting/video_topics')
    # graphing_topic_list = rospy.get_param('/ros_toolkit/plotting/graphing_topics')
    queue_size = rospy.get_param('/ros_toolkit/plotting/queue_size')
    slop_time = rospy.get_param('/ros_toolkit/plotting/slop_time')

    screen_shot_plotting = ScreenShotPlotting(snapshot_topic_list, int(queue_size), int(slop_time))
    # video_plotting = ContinuousPlotting(video_topic_list, int(queue_size), int(slop_time))


    # graphing = Graphing(graphing_topic_list, int(queue_size), int(slop_time))
    
    
    while not rospy.is_shutdown():   
        #we are in python2.7! 
        try:  
            input_command = str(raw_input())
            PlottingManager.execute_commands(input_command)
        except KeyboardInterrupt, Exception:
            rospy.signal_shutdown("CTR-C recieved")
            
            clear_records = str(raw_input("Would you like to clear records just made: {} [y/n]".format(PlottingManager.get_current_record_folder())))
            if clear_records == "y":
                print("Clearing records")
                PlottingManager.clear_record()

            PlottingManager.finalise()


        

    
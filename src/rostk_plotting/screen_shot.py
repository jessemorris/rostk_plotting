from rostk_plotting.transforms import *
from rostk_plotting.plotting_callbacks import PlottingCallbacks
from rostk_plotting.plotting_manager import PlottingManager, attribute_event

import cv2
import numpy as np
from threading import Lock


class ScreenShotPlotting(PlottingManager, PlottingCallbacks):

    def __init__(self, topic_list, queue_size, slop_time):
        PlottingCallbacks.__init__(self)
        PlottingManager.__init__(self, topic_list, queue_size, slop_time, self)

        self.screen_cap_flag = False

        #TODO: wrap this for each topic subscirbed to so we can save each file as topic_name + id
        self.image_id = 0
        self.point_cloud_id = 0
        self.pc_lock = Lock()

    def post_callback(self):
        self.screen_cap_flag = False

    def on_shutdown(self):
        print("Goodbye")

    def parse_command(self, command_string):
        if command_string == "sc":
            rospy.loginfo("Taking screenshot")
            self.screen_cap_flag = True


    @attribute_event("screen_cap_flag")
    def camera_info_callback(self, data):
        pass

    @attribute_event("screen_cap_flag")
    def point_cloud_callback(self, data):
        points = point_cloud_to_ndarray(data)
        self.pc_lock.acquire()
        np.save(ScreenShotPlotting.get_current_record_folder() + str(self.point_cloud_id) + ".npy", points)
        print("Saved {} points to file".format(len(points)))
        self.pc_lock.release()
        self.point_cloud_id+=1



        
    @attribute_event("screen_cap_flag")
    def image_callback(self, data):
        input_image = image_msg_to_np(data)
        cv2.imwrite(ScreenShotPlotting.get_current_record_folder() + str(self.image_id) + ".png", input_image)
        print("Written image")
        self.image_id+=1

from rostk_plotting.transforms import *
from rostk_plotting.plotting_callbacks import PlottingCallbacks
from rostk_plotting.plotting_manager import PlottingManager, attribute_event, _parse_datatype_path

import cv2

class ContinuousPlotting(PlottingManager, PlottingCallbacks):
    #there is no way to check which message comes from which topic so
    # we can only have unique msg types to contuously record otherwise the messages
    # will get jumbpled up

    def __init__(self, topic_list, queue_size, slop_time):
        self._check_unique_msgs(topic_list)
        PlottingCallbacks.__init__(self)
        PlottingManager.__init__(self, topic_list, queue_size, slop_time, self)

        self.start_recording = False
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')

        self._video_written = False
        self._image_topic = None #can only set this inside the image callback function
        self._images = []

    def post_callback(self):
        # self.screen_cap_flag = False
        # save all stored data as video file
        pass

    def on_shutdown(self):
        if self._video_written and self._image_topic and len(self._images) > 1:
            video_writer = cv2.VideoWriter(ContinuousPlotting.get_current_record_folder() + self._image_topic + '.avi',self.fourcc, 20.0, (640,480))
            for image in self._images:
                video_writer.write(image)
            video_writer.release()

    def parse_command(self, command_string):
        if command_string == "start":
            rospy.loginfo("Starting video record")
            self.start_recording = True
            self._video_written = True

        if command_string == "stop":
            rospy.loginfo("Ending video record")
            self.start_recording = False

    @attribute_event("start_recording")
    def camera_info_callback(self, data):
        pass

    @attribute_event("start_recording")
    def image_callback(self, data):
        if not self._image_topic:
            #set topic as video outputname
            topic_list = self.get_topic_on_callback()
            assert(len(topic_list) == 1)
            self._image_topic = topic_list[0].replace("/", "_")
            print(self._image_topic)
        self._images.append(image_msg_to_np(data))

    def _check_unique_msgs(self, topic_list):
        msg_types = []
        for topic_map in topic_list:
            assert(len(topic_map.keys()) == 1)
            topic_name = topic_map.keys()[0]
            type_path = topic_map.values()[0]

            module_name, data_class = _parse_datatype_path(type_path)
            if data_class in msg_types:
                raise Exception("Cannot subscribe to multiple {} topics for continuous plotting.".format(data_class))
            msg_types.append(data_class)

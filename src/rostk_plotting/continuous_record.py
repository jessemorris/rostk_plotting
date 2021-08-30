from rostk_plotting.transforms import *
from rostk_plotting.plotting_callbacks import PlottingCallbacks
from rostk_plotting.plotting_manager import PlottingManager, attribute_event, _parse_datatype_path, get_topic_from_msg, convert_topic_to_file_name

import cv2

class ContinuousPlotting(PlottingManager, PlottingCallbacks):

    def __init__(self, topic_list, queue_size, slop_time):
        PlottingCallbacks.__init__(self)
        PlottingManager.__init__(self, topic_list, queue_size, slop_time, self)

        self.start_recording = False
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')

        # maps a topic to an array of images to be saved later by the video writer
        self._topic_images_map = {}

        for topic_dict in topic_list:
            topics = list(topic_dict.keys())[0]
            self._topic_images_map[topics] = []
            rospy.loginfo("Made video array for {}".format(topics))

        self._video_written = False
        self._image_topic = None #can only set this inside the image callback function
        self._images = []

    def post_callback(self):
        # self.screen_cap_flag = False
        # save all stored data as video file
        pass

    def on_shutdown(self):
        if self._video_written:
            for topic, video_streams in self._topic_images_map.items():
                rospy.loginfo("Writing videos for topic: {}".format(topic))

                #get size of video to set the size of the video writer -> we assume the image sizes stay the same 
                first_image_size = video_streams[0].shape
                video_writer = cv2.VideoWriter(ContinuousPlotting.get_current_record_folder() + convert_topic_to_file_name(topic) + '.avi',
                    self.fourcc, 20.0, (first_image_size[1],first_image_size[0]))
                for image in video_streams:
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
        topic = get_topic_from_msg(data)
        rospy.loginfo("Topic {}".format((topic)))
        self._topic_images_map[topic].append(image_msg_to_np(data))


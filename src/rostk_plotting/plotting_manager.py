from threading import Thread, Condition, RLock
import message_filters
import roslib
import rospy
import cv2
import signal
from datetime import datetime
from functools import wraps
from shutil import rmtree
import inspect
import functools
from geometry_msgs.msg import Twist
from rostk_plotting.dynamic_import import DynamicImport
from rostk_plotting.plotting_callbacks import PlottingCallbacks
import sys, select, termios, tty, os
import rospkg


rospack = rospkg.RosPack()
package_path = rospack.get_path("rostk_plotting")


record_path = package_path + "/records/"



#should be the name of some variable that, when true, triggers the callback function
def attribute_event(attribute_flag):
    def _check_signal_flag(f):
        @functools.wraps(f)
        def wrapper(self, *args):
            try:
                flag = getattr(self, attribute_flag)
                #only triggers function if true
                if flag:
                    return f(self, *args)
            except AttributeError as e:
                rospy.logwarn(e)
        return wrapper
    return _check_signal_flag

def _parse_datatype_path(type_path):
    # path will be like module1.module2.Class
    # I think most of the time it will be rospack.msg.Class
    #module_path will be module1.module2 and dataclass will be Class
    path_list = type_path.split(".")
    module_path = ".".join(path_list[:-1])
    data_class = path_list[-1]
    return module_path, data_class

def _classname_from_module(class_instance):
    #takes a full class name such as <class 'sensor_msgs.msg._CameraInfo.CameraInfo'>
    # and gets the class name only (ie. the name after the final ".")
    # eg output here will be CameraInfo
    msg_type = type(class_instance)
    class_instance_str = str(msg_type)
    module_list = class_instance_str.split(".")
    class_name = module_list[-1][:-2]
    return class_name

def _get_info_from_topic_map(topic_map):
    #topic map is conofig info {topic:msgtype}
    #helper function to extract info from topic map (which is used as the 
    # iterator for the self.topic_list attribute)
    #returns topic, module_name and msg_class_name
    assert(len(topic_map.keys()) == 1)
    topic_name = topic_map.keys()[0]
    type_path = topic_map.values()[0]

    module_name, data_class = _parse_datatype_path(type_path)
    return topic_name, module_name, data_class


def get_topic_from_msg(msg):
    header = msg._connection_header
    if 'topic' in header:
        return header['topic']
    else:
        return ''

def convert_topic_to_file_name(topic):
    """[Converts a namespaced topic to a string that can be used as a file name.
    Eg. camera/rgb/image_raw -> camera_rgb_image_raw]

    Args:
        topic ([str]): [input topic name]

    Returns:
        [str]: [topic name as an okay file format]
    """
    return topic.replace("/", "_")


class PlottingManager():
    #list of all the managers so we can call parse command on all of them
    __manager_list = []

    _current_record_folder = None

    def __init__(self, topic_list, queue_size, slop_time, plotting_callbacks = None):
        PlottingManager.__manager_list.append(self)
        self._topic_list = topic_list
        self.dynamic_import = DynamicImport()
        self.subscriber_list = []
        #dictionary of msg type names (eg. "CameraInfo") -> to a function that will be called and act as
        # the method of saving this data one recieved. New methods can be added and overwritten for specific cases
        self.saving_methods = {}
        self.queue_size = queue_size
        self.slop_time = slop_time
        if plotting_callbacks:
            #use the provided PlottingCallbacks implenetation
            assert(isinstance(plotting_callbacks, PlottingCallbacks))
            self.plotting_callbacks = plotting_callbacks
        else:
            #use the default one
            self.plotting_callbacks = PlottingCallbacks()
        self.create_subscribers()

        #TODO: dont make time synchronous -> just make new subscribers and make callback
        #get the topic name and then callback the custom callback with the data and the topic name
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(self.subscriber_list, self.queue_size, self.slop_time, allow_headerless=True)
        self.time_synchronizer.registerCallback(self.callback)

        self.saving_methods.update(self.plotting_callbacks.get_default_callbacks())
        # self._continue_listening = True
        # self._condition = Condition()

        if self.queue_size != 0:
            self._key_timeout = 1.0 / self.queue_size
        else:
            self._key_timeout = None

    def create_subscribers(self):
        for topic_map in self._topic_list:
            
            topic_name, module_name, data_class = _get_info_from_topic_map(topic_map)
            subscriber_msg_type = self.dynamic_import(module_name, data_class)
            subscriber_msg_name = subscriber_msg_type.__name__
            sub = message_filters.Subscriber(topic_name, subscriber_msg_type)
            rospy.loginfo("Subscribing to topic: {} [{}]".format(topic_name, subscriber_msg_name))
            self.subscriber_list.append(sub)
            self.saving_methods[subscriber_msg_name] = None

    def get_number_subscribers(self):
        return len(self.subscriber_list)

    def get_subscribed_topics(self):
        topics = []
        for topic_map in self._topic_list:
            topic_name, module_name, data_class = _get_info_from_topic_map(topic_map) 
            topics.append(topic_name)
        return topics

    def post_callback(self):
        ##called immdiately after the callback function so we can reset any flags
        raise NotImplementedError

    def on_shutdown(self):
        #called when the user closes the program - can be used to write all collectively saved data to disk
        raise NotImplementedError

    def callback(self, *args):
        # the callback for when we get all the time synchronizer msgs
        for msgs in args:
            self._call_saving_method(msgs)

        self.post_callback()
        

    def add_saving_method(self, msg_name, func):
        #msg name should be a string and be a type that the time sync is subscribing to eg "Image", "CameraInfo"
        # func should be any function that takes a single param (the data). This is used 
        # to do some operation on the data such as save to a data stream or write to memory
        if msg_name in self.saving_methods:
            self.saving_methods[msg_name] = func
        else:
            rospy.logwarn("Message name {} is not a subscribed datatype.".format(msg_name))

    def parse_command(self, command_string):
        raise NotImplementedError


    def _call_saving_method(self, msg):
        ## calls the overwritten saving method (usually defined in plotting callbacks)
        # by finding the name of the type of msg and its associated callback
        msg_class_name = _classname_from_module(msg)
        if msg_class_name in self.saving_methods:
            saving_callback = self.saving_methods[msg_class_name]
            saving_callback(msg)
        else:
            rospy.logwarn("{} has not saving method attached".format(msg_class_name))


    @staticmethod
    def make_new_record():
        assert(PlottingManager._current_record_folder == None)
        today = datetime.now()
        PlottingManager._current_record_folder = record_path + today.strftime("%m:%d:%Y-%H:%M:%S")
        print("Current results directionary: {}".format(PlottingManager._current_record_folder))
        os.mkdir(PlottingManager._current_record_folder)

    @staticmethod
    def clear_record():
        if PlottingManager._current_record_folder:
            rmtree(PlottingManager._current_record_folder)
        

    @staticmethod
    def set_current_record_folder():
        if PlottingManager._current_record_folder is None:
            PlottingManager._make_new_record()

        return PlottingManager._current_record_folder + "/"


    @staticmethod
    def get_current_record_folder():
        return PlottingManager._current_record_folder + "/"


    @staticmethod
    def execute_commands(command_string):
        for manager in PlottingManager.__manager_list:
            manager.parse_command(command_string)

    @staticmethod
    def finalise():
        for manager in PlottingManager.__manager_list:
            manager.on_shutdown()

        
PlottingManager.make_new_record()
from threading import Thread, Condition
import message_filters
import roslib
import rospy
import cv2
import signal
from datetime import datetime
from functools import wraps


from geometry_msgs.msg import Twist
from rostk_plotting.dynamic_import import DynamicImport
from rostk_plotting.plotting_callbacks import PlottingCallbacks
import sys, select, termios, tty, os
import rospkg

# def key_event(signal, func=None):
#     def somedec_outer(fn):
#         @wraps(fn)
#         def somedec_inner(*args, **kwargs):
#             print(signal)
#             print(func)
#             response = fn(*args, **kwargs)
#             return response
#         return somedec_inner
#     return somedec_outer



def key_event(attribute):
    def _check_authorization(f):
        def wrapper(self, *args):
            print getattr(self, attribute)
            return f(self, *args)
        return wrapper
    return _check_authorization

    
class PlottingManager():

    def __init__(self, topic_list, queue_size, slop_time, plotting_callbacks = None):
        self.topic_list = topic_list
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
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(self.subscriber_list, self.queue_size, self.slop_time)
        self.time_synchronizer.registerCallback(self.callback)

        self.saving_methods.update(self.plotting_callbacks.get_default_callbacks())
        # self._continue_listening = True
        # self._condition = Condition()

        if self.queue_size != 0:
            self._key_timeout = 1.0 / self.queue_size
        else:
            self._key_timeout = None

        
        # self.key_notifer_thread = Thread(target=self._key_notifer_worker)

    def create_subscribers(self):
        for topic_map in self.topic_list:
            assert(len(topic_map.keys()) == 1)
            topic_name = topic_map.keys()[0]
            type_path = topic_map.values()[0]

            module_name, data_class = self._parse_datatype_path(type_path)
            subscriber_msg_type = self.dynamic_import(module_name, data_class)
            subscriber_msg_name = subscriber_msg_type.__name__
            sub = message_filters.Subscriber(topic_name, subscriber_msg_type)
            rospy.loginfo("Subscribing to topic: {} [{}]".format(topic_name, subscriber_msg_name))
            self.subscriber_list.append(sub)
            self.saving_methods[subscriber_msg_name] = None


    def callback(self, *args):
        for msgs in args:
            self._call_saving_method(msgs)
        

    def add_saving_method(self, msg_name, func):
        #msg name should be a string and be a type that the time sync is subscribing to
        # func should be any function that takes a single param (the data). This is used 
        # to do some operation on the data such as save to a data stream or write to memory
        if msg_name in self.saving_methods:
            self.saving_methods[msg_name] = func
        else:
            rospy.logwarn("Message name {} is not a subscribed datatype.".format(msg_name))

    def parse_command(self, command_string):
        raise NotImplementedError

    def _parse_datatype_path(self, type_path):
        # path will be like module1.module2.Class
        # I think most of the time it will be rospack.msg.Class
        path_list = type_path.split(".")
        module_path = ".".join(path_list[:-1])
        data_class = path_list[-1]
        return module_path, data_class

    def _classname_from_module(self, class_instance):
        #takes a full class name such as <class 'sensor_msgs.msg._CameraInfo.CameraInfo'>
        # and gets the class name only (ie. the name after the final ".")
        class_instance_str = str(class_instance)
        module_list = class_instance_str.split(".")
        class_name = module_list[-1][:-2]
        return class_name


    def _call_saving_method(self, msg):
        msg_type = type(msg)
        msg_class_name = self._classname_from_module(msg_type)
        if msg_class_name in self.saving_methods:
            saving_callback = self.saving_methods[msg_class_name]
            saving_callback(msg)
        else:
            rospy.logwarn("{} has not saving method attached".format(msg_class_name))

    # def stop_key_notifier(self):
    #     self._continue_listening = False
    #     rospy.sleep(1)
    #     self.key_notifer_thread.join()
        

#!/usr/bin/env python
import threading
import message_filters
import roslib
import rospy
import cv2


from geometry_msgs.msg import Twist
from rostk_plotting.dynamic_import import DynamicImport
from rostk_plotting.plotting_callbacks import PlottingCallbacks
from copy import deepcopy
import sys, select, termios, tty

msg = ""


moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key





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

if __name__=="__main__":
    # settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("rostk_plotting")
    snapshot_topic_list = rospy.get_param('/ros_toolkit/plotting/snapshot_topics')
    queue_size = rospy.get_param('/ros_toolkit/plotting/queue_size')
    slop_time = rospy.get_param('/ros_toolkit/plotting/slop_time')
    manager = PlottingManager(snapshot_topic_list, int(queue_size), int(slop_time))

    # di = DynamicImport()

    # obj = di("sensor_msgs.msg", "CameraInfo")
    # print(obj)
    # print(obj.dir())
    rospy.spin()

    # try:
    #     pub_thread.wait_for_subscribers()
    #     pub_thread.update(x, y, z, th, speed, turn)

    #     print(msg)
    #     print(vels(speed,turn))
    #     while(1):
    #         key = getKey(key_timeout)
    #         if key in moveBindings.keys():
    #             x = moveBindings[key][0]
    #             y = moveBindings[key][1]
    #             z = moveBindings[key][2]
    #             th = moveBindings[key][3]
    #         elif key in speedBindings.keys():
    #             speed = speed * speedBindings[key][0]
    #             turn = turn * speedBindings[key][1]

    #             print(vels(speed,turn))
    #             if (status == 14):
    #                 print(msg)
    #             status = (status + 1) % 15
    #         else:
    #             # Skip updating cmd_vel if key timeout and robot already
    #             # stopped.
    #             if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
    #                 continue
    #             x = 0
    #             y = 0
    #             z = 0
    #             th = 0
    #             if (key == '\x03'):
    #                 break
 
    #         pub_thread.update(x, y, z, th, speed, turn)

    # except Exception as e:
    #     print(e)

    # finally:
    #     pub_thread.stop()

    #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    
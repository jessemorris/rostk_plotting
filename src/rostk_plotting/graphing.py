from rostk_plotting.transforms import *
from rostk_plotting.plotting_callbacks import PlottingCallbacks
from rostk_plotting.plotting_manager import PlottingManager, attribute_event, get_topic_from_msg

import rospy
# import matplotlib
# matplotlib.use('GTKAgg')
# import matplotlib.pyplot as plt

import numpy as np
import cv2

__graphable_types = [int, float]

def get_graphable_data(data_dict):
    data = {}
    for key, values in data_dict.items():
        if type(values) in __graphable_types:
            data[key] = values

    return data, len(data)

class Grapher():


    def __init__(self, plot_width, plot_height, num_plot_values):
        self.width = plot_width
        self.height = plot_height
        self.color_list = [(255, 0 ,0), (0, 250 ,0),(0, 0 ,250),
            (0, 255 ,250),(250, 0 ,250),(250, 250 ,0),
            (200, 100 ,200),(100, 200 ,200),(200, 200 ,100)]
        self.color  = []
        self.val = []
        self.plot = np.ones((self.height, self.width, 3))*255

        for i in range(num_plot_values):
            self.color.append(self.color_list[i])

    # Update new values in plot
    def multiplot(self, val, label = "plot"):
        self.val.append(val)
        while len(self.val) > self.width:
            self.val.pop(0)

        self.show_plot(label)

    # Show plot using opencv imshow
    def show_plot(self, label):
        self.plot = np.ones((self.height, self.width, 3))*255
        cv2.line(self.plot, (0, int(self.height/2) ), (self.width, int(self.height/2)), (0,255,0), 1)
        for i in range(len(self.val)-1):
            for j in range(len(self.val[0])):
                i = int(i * 10)
                j = int(j * 10)
                cv2.line(self.plot, (i, int(self.height/2) - self.val[i][j]), (i+1, int(self.height/2) - self.val[i+1][j]), self.color[j], 1)

        cv2.imshow(label, self.plot)
        cv2.waitKey(10)




class Graphing(PlottingManager):

    def __init__(self, topic_list, queue_size, slop_time):
        PlottingManager.__init__(self, topic_list, queue_size, slop_time)
        self.add_saving_method("Point", self.point_callback)



        self.float_time = None
        self.is_first = True

        self.number_of_graphs = self.get_number_subscribers()
        print(self.number_of_graphs)
        self.subscribed_topics = self.get_subscribed_topics()

        self.topic_plot_map = {}

        assert(len(self.subscribed_topics) == self.number_of_graphs)

        #for now only go in cols - can optimize this later
        # self.figs, self.axes = plt.subplots(1, self.number_of_graphs)

        # if self.number_of_graphs > 1:
        #     for i in range(self.number_of_graphs):
        #         plots = Plots(self.axes[i])
        #         self.topic_plot_map[self.subscribed_topics[i]] = plots
        # else:
        #     plots = Plots(self.axes)
        #     self.topic_plot_map[self.subscribed_topics[0]] = plots

        self.grapher = Grapher(400, 200, self.number_of_graphs)


    def post_callback(self):
        pass

    def on_shutdown(self):
        pass

    def parse_command(self, command_string):
        pass

    def point_callback(self, data):
        topic = get_topic_from_msg(data)
        print(topic)
        self.float_time = rospy.get_rostime().secs
        data, size = get_graphable_data(ros_msg_to_dict(data))

        
        self.grapher.multiplot([data['x'], data['y']])




# pyROS Toolkit

## Plotting Module

A small rospy interface that allows ros topic data to be stored/recorded from keyboard input for later analysis or use in figures.

The module is aimed to be as custom and dynamic as possible. Only  modification of the config.yaml file for the most common applications (such as screenshotting images). The interface allows easy customisation of saving functions for any type of ROS msg data.

Modifying the topics in the config file will subscribe the node to the specified topics (either for the purpose of screencapping from each topic, or recording the video frames and exporting as a video)

### Basic Commands:
 - Type "sc" to screen shot any topics under _snapshot_topics_
 - Type "start to begin recording frames from _video_topics_. Type "stop" to stop recording. Upon ctrl-C, this video will be
 written to file using the topic name of the video as the filename. 

### Requirements
- rospy
- opencv
- ros-melodic-rospy-message-converter (non-default python module)
- ros-numpy

### Config and basic use
The module subscribes to topics specified in the __rostk_plotting_config.yaml__. Thse should be in the form:

```{topic:msg_module.msg.Class}``` 
eg ```{"/gmsl/A1/camera_info": "sensor_msgs.msg.CameraInfo"}```
where the topic is a string and the module and class define the ROS msg type that is being published by the master.

You can have as many of these as you want defined in the config.

NOTE: you may need to add extra dependancies to the package.xml and CMakeLists.txt for non-standard ROS msgs.

These should be stored under a header dictiotionary (see the default config file) that will be passed to the __PlottingManager__.

### PlottingManager ###

This managers all the ros callbacks and custom saving methods for each topic. Each msg type will have its own callback (see plotting_callbacks.py for basic callbacks). The rospy.subscriber triggers a callback in the plotting manager which checks the datatype of each incoming message. The linked saving methods are then called using each data msg as input. Implementing and linking one of these methods allows the data to be saved however the user wants - ie writing to an image or saving to a txt file depending.

By inheriting from PlottingManager and PlottingCallbacks you can overwrite these basic methods or one method yourself. See __ScreenShotPlotting__ as a default class that is implemented to take screenshots of topics. Specific functions must be implemented to form the interface:

#### post_callback()
This method is called immediately after all the custom save methods are called. In this way any flags that have been set by the user can be reset.

#### on_shutdown()
Is called just before the program ends. This can be used to save any accumulated data to file - eg sequence of images, collected by each callback, can be saved as a video/gif etc at this time.

#### parse_command()
This method is called when the user enter input. Any custom flags can be set depending on this user input. This is required due to the event flag mechanism that triggers the saving method callbacks.

#### Event Triggering
Any custom saving method can be told to trigger exclusively on flag being set to true. 

```python
class RosRecording(PlottingManager):
    ...

    self.trgger_flag = False

    @attribute_event("trgger_flag")
    def custom_saving_method(self, data):
        # save data here

```

Adding the ```attribute_event``` decorator tells the function to only trigger when the variable with the str name "trigger_flag" is set to True. This should happen in the parse_command function but can happen anywhere. 
```python
def parse_command(self, command):
    # some custom command parsing from stdin
    ...
    self.trigger_flag = True

```


To avoid the callback being called continuously, the flag should then be set to false in the ```post_callback()``` function. If you want to the function to trigger on every ros callback, leave off the decorator. 

#### Custom Saving commands
You can define some custom function that will save some specific data

1. Add topic and msg type to config
2. Write custom class that inherits from __PlottingManager__ (and __PlottingCallbacks__ if you want the default callbacks already defined)
3. Write custom function that takes your data and add it to the plotting manager using `add_saving_method(<msg class name>, <callback>)`


## Screen Shot 
This class is implemented in __plotting_main.py__. It currently only has functions to listen to ```sensor_msgs.msg.Image/CameraInfo```. 
Type __sc__ (ie. screenshot) and all listening topics will be saved in the __results__ folder. There is currently no method to create subdirectories and order the stored data more coherently. 

## TODO
- add custom sub directories for each manager class
- make sure file names do not overlap (unless in different folders)
- maybe add command line tool if this does not take too much time (to delete all results packages eg)

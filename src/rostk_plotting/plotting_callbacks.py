class PlottingCallbacks():
    # define some default methods to call - can be overwritten by adding your own 
    # methods by overwriting this class or just adding your own function with
    # plotting_manager.add_saving_method
    def __init__(self):

        self.default_callbacks = {"CameraInfo": self.camera_info_callback,
                                "Image": self.image_callback
                                
                            }
        

    def get_default_callbacks(self):
        return self.default_callbacks

    def camera_info_callback(self, data):
        print("In camera info")


    def image_callback(self, data):
        print("in image callback")
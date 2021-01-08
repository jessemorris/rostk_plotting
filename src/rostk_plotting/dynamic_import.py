import imp 
import sys 
import rospy
import importlib



class DynamicImport():
    def __init__(self):
        pass

    def __call__(self, module_name, class_name):
        module = importlib.import_module(module_name)
        return getattr(module, class_name)
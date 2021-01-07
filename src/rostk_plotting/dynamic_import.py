import imp 
import sys 
import rospy
import importlib



class DynamicImport():
    def __init__(self):
        pass

    # def __call__(self, module_name, class_name):
    #     try: 
    #         fp, path, desc = imp.find_module(module_name) 
          
    #     except ImportError: 
    #         rospy.logerr("Module not found: " + module_name) 
            
    #     try: 
    #         class_instance = imp.load_module("% s.% s" % (module_name, 
    #                                             class_name),  
    #                                 fp, path, desc) 
            
    #     except Exception as e: 
    #         rospy.logerr(e) 
            
    #     return class_instance 

    def __call__(self, module_name, class_name):
        module = importlib.import_module(module_name)
        return getattr(module, class_name)
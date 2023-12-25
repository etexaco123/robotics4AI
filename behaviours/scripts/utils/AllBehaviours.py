import rospy
import sys
import inspect

import importlib

class AllBehaviours(object):

    class __impl:
        all_behaviours = []

        def __init__(self):
            self.load_behaviours()

        def load_behaviours(self):

            if not rospy.has_param("main_behaviour"):
                print 'main_behaviour parameter not found, exiting...'
                exit(0)

            main_behaviour_name = str(rospy.get_param("main_behaviour"))
            module_name = main_behaviour_name
            behaviour = self.create_class(main_behaviour_name, module_name)

            self.all_behaviours.append(behaviour)

            sub_behaviour_names = rospy.get_param("sub_behaviours", [])

            for sub in sub_behaviour_names:
                behaviour = self.create_class(sub, module_name)
                self.all_behaviours.append(behaviour)

        def create_class(self, class_name, module_name):
            module = importlib.import_module("behaviours." + module_name + "." + class_name)
            
            for name, obj in inspect.getmembers(module):
                
                if inspect.isclass(obj) and name == class_name:
                    behaviour = obj(self)
                    return behaviour

        def get_behaviour(self, name):

            for behaviour in self.all_behaviours:
                if behaviour.__class__.__name__ == name:
                    return behaviour

            return None

        def get_all_behaviours(self):
            return self.all_behaviours

    __instance = __impl()

    def __getattr__(self, item):
        return getattr(self.__instance, item)

    def __setattr__(self, key, value):
        return setattr(self.__instance, key, value)



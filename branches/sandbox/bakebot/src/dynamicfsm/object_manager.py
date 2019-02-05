#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')

class ObjectManager():
    def __init__(self):
        pass

    def get_str_object(self, recipe_ingred_name):
        # return the object from the btom that most closely matches the recipe ingred 
        # based on some criterion
        pass

    def get_object_from_str(self, object_name):
        # return the object from the btom that has this name
        pass

#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import sys
import pickle
import utilities.recipe_compiler
from dynamicfsm.ingredient_filter import *
from bakebot.srv import *

class BakebotClient():
    @staticmethod
    def execute_recipe(recipe, debug=False):
        try:
            print 'waiting for service...'
            rospy.wait_for_service('execute_recipe')
            print 'calling service...'
            execute_recipe = rospy.ServiceProxy('execute_recipe', RoboRecipe)
            result = execute_recipe(recipe, debug)
            return result
        except rospy.ServiceException, e:
            print 'service call failed: %s' % e
            return False

    @staticmethod
    def get_table_items():
        try:
            rospy.wait_for_service('get_table_items')
            get_table_items = rospy.ServiceProxy('get_table_items', BakebotTableItems)
            result = get_table_items()
            return _process_table_items(result)
        except rospy.ServiceException, e:
            print 'service call failed: %s' % e
            return None

    @staticmethod
    def _process_table_items(item_str):
        return pickle.loads(item_str)


class BakebotExecutive():
    def __init__(self, ingredient_filter, ingreds_in_mb=None, minimize_mixes=False):
        self.recipe_service = rospy.Service('execute_recipe', RoboRecipe, self.execute_recipe)
        self.object_service = rospy.Service('get_table_items', BakebotTableItems, self.get_table_items)
        self.ingredient_filter = ingredient_filter
        self.detection_performed = False
        self.ingreds_in_mb = ingreds_in_mb
        self.minimize_mixes = minimize_mixes

    def spin(self):
        rospy.spin()

    def execute_recipe(self, req):
        rospy.loginfo('received a recipe to execute')
        print req.recipe
        fsm, user_instructions = utilities.recipe_compiler.generate_fsm(req.recipe, self.ingredient_filter, debug=req.debug, do_detection=(not self.detection_performed), ingreds_in_mb=self.ingreds_in_mb, minimize_mixes=self.minimize_mixes)
        print 'got the fsm'
        status = False
        try:
            status = fsm.execute()
        finally:
            print 'finally: ', status
            return RoboRecipeResponse(status)

    def get_table_items(self, req):
        rospy.loginfo('received a request for the items on the table')
        btom = BroadTabletopObjectManager.get_broad_tabletop_object_manager()
        if self.load_filename == None:
            btom.do_broad_tabletop_object_detection()
            self.load_filename = 'detectedobj.log'
            btom.save_detected_objects_to_file(self.load_filename)
        else:
            status = btom.load_detected_objects_from_file(self.load_filename)
            if not status:
                return BakebotTableItems('None')
# include flags so that we don't include the load/scan states in the final fsm if this has been called prior to the recipe bieng executed
        btom.filter_ingredients(ingredient_filter = self.ingredient_filter)
        matched_set = self.ingredient_filter.get_matched_set()
        matched_set.append((btom.cookie_sheet, 'cs'))
        matched_set.append((btom.mixing_bowl, 'mb'))
        items = list()
        for matched_pair in matched_set:
            name, obj = matched_pair
            #TODO: make sure this is in the right frame
            x = obj.pose.pose.position.x
            y = obj.pose.pose.position.y
            z = obj.pose.pose.position.z
            items.append((name, (x, y, z)))
        self.detection_performed = True
        return BakebotTableItems(self._encode_table_items(items))

    def _encode_table_items(self, items):
        return pickle.dumps(items)


if __name__ == '__main__':
    rospy.init_node('bakebot_executive')
    if len(sys.argv) > 1:
        if sys.argv[1] == 'afghans':
            print 'setting flags for chocolate afghans'
            bakebot = BakebotExecutive(ClockwiseIngredientFilter(), ingreds_in_mb=['butter'], minimize_mixes=True)
        elif sys.argv[1] == 'sugar':
            print 'setting flags for sugar cookies'
            bakebot = BakebotExecutive(ClockwiseIngredientFilter(), ingreds_in_mb=['eggs', 'oil', 'vanilla'], minimize_mixes=True)
        elif sys.argv[1] == 'verbatim':
            print 'setting flags for verbatim recipe execution (no mix reduction or ingredients in mb)'
            bakebot = BakebotExecutive(ClockwiseIngredientFilter(), ingreds_in_mb=[], minimize_mixes=False)
        else:
            print 'did not recognize command line recipe argument'
            bakebot = BakebotExecutive(ClockwiseIngredientFilter(), minimize_mixes=True)
    else:
        bakebot = BakebotExecutive(ClockwiseIngredientFilter(), minimize_mixes=True)
    print 'service initialized'
    bakebot.spin()

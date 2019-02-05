#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')

class IngredientFilter():
    def filter_ingredients(self, ingredients):
        '''
        filters the ingredients to prepare for a mapping
        '''
        raise NotImplementedError

    def get_ingredient_iterator(self):
        '''
        returns an iterator over the filtered ingredients
        '''
        raise NotImplementedError

    def get_ingredient(self, name):
        '''
        returns the ingredient corresponding to the name
        '''
        raise NotImplementedError

    def get_matched_set(self):
        '''
        returns a list of tuples, (name, detected_obj)
        '''
        raise NotImplementedError



class ClockwiseIngredientFilter(IngredientFilter):
    
    def __init__(self):
        self.detected_ingredients = None
        pass

    def filter_ingredients(self, ingredients):
        sorted_by_y = ingredients
        #TODO this has to be changed
        if len(sorted_by_y) != 4 and False:  # indicating not a full test table
            print 'not a full table...'
            for obj in sorted_by_y:
                print obj.pose.pose.position.x, obj.pose.pose.position.y
            self.detected_ingredients = sorted_by_y
        else:
            sorted_by_x = sorted(sorted_by_y, key = lambda obj: obj.pose.pose.position.x)
            # take the first two out (first row)
            first_row = [sorted_by_x[0], sorted_by_x[1]]
            for obj in first_row:
                sorted_by_x.remove(obj)
            second_row = sorted_by_x
            # we want the first row sorted right to left
            first_row = sorted(first_row, key = lambda obj: obj.pose.pose.position.y)
            # we want the second row sorted left to right
            second_row = sorted(second_row, key = lambda obj: -obj.pose.pose.position.y)
            self.detected_ingredients = list()
            self.detected_ingredients.extend(first_row)
            self.detected_ingredients.extend(second_row)
            for obj in self.detected_ingredients:
                print obj.pose.pose.position.x, obj.pose.pose.position.y

    def get_ingredient_iterator(self):
        return iter(self.detected_ingredients)

    def get_ingredient(self, name):
        if self.detected_ingredients == None:
            return None
        if name == 'sugar':
            return self.detected_ingredients[0]
        elif name == 'cocoa' or name == 'salt':
            return self.detected_ingredients[1]
        elif name == 'flour':
            return self.detected_ingredients[2]
        else:
            return self.detected_ingredients[3]
        
    def get_matched_set(self):
        ret = list()
        ingreds = ['sugar', 'cocoa', 'flour', 'krispies']
        for ingred in ingreds:
            ret.append((ingred, self.get_ingredient(ingred)))
        return ret

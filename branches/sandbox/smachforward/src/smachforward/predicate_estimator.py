#!/usr/bin/env python
import roslib
roslib.load_manifest('smachforward')

class PredicateEstimator():
    def get_in_keys(self):
        '''
        returns all of the input keys this needs to
        parse the userdata into predicate states
        '''
        raise NotImplementedError

    def generate_problem_str(self, userdata, predicates, goal_str):
        '''
        Takes the userdata and turns it into a string list of predicates
        followed by the goal_str
        '''
        raise NotImplementedError

    def get_predicate_value(self, userdata, predicate_str):
        '''
        returns True or False based on evaluation of the predicate_str
        given robot state and the userdata.
        @param predicate_str ex: (reset ?a) or (reset ?rarm) or (carry ?larm ?b)
        '''
        raise NotImplementedError

    def rename_and_expand_predicate_list(self, predicate_list):
        '''
        renames the variables in the predicates when appropriate, expanding to
        eliminate ambiguity.  ex: (reset ?a) --> (reset ?larm) and (reset ?rarm)
        returns a list probably longer than predicate_list
        '''
        raise NotImplementedError

    def check_preconditions(self, fstate, userdata):
        '''
        returns the boolean evaluation of the precondition predicate expression for fstate
        '''
        _check_precondition_or_effect(self, fstate, userdata, check_effect=False)

    def check_effect(self, fstate, userdata):
        '''
        returns the boolean evaluation of the effect predicate expression for fstate
        '''
        _check_precondition_or_effect(self, fstate, userdata, check_effect=True)

#    def _check_precondition_or_effect(self, fstate, userdata, check_effect):
#        predicate_str = fstate.get_effect_str() if check_effect else fstate.get_precondition_str()
#    
#    def _rename_and_evaluate_predicate(self, predicate, userdata):
#        # want to rename variables before 
#        lst = self.rename_and_expand_predicate_list([predicate])
#        return self.get_predicate_value(lst[0])

        

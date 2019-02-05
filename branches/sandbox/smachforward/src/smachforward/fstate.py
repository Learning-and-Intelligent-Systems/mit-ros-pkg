#!/usr/bin/env python
import roslib
roslib.load_manifest('smachforward')
import smach


class FState(smach.State):
    def __init__(self, input_keys = [], output_keys = []):
        self.in_keys = input_keys
        self.out_keys = output_keys
        smach.State.__init__(self, outcomes=['success', 'failure'], 
                             input_keys = self.in_keys,
                             output_keys = self.out_keys)
        self.parametrized = False

    def get_input_keys(self):
        return self.in_keys

    def get_output_keys(self):
        return self.out_keys

    def get_name_str(self):
        return self.__class__.__name__

    def get_params_str(self):
        raise NotImplementedError

    def get_precondition_str(self):
        raise NotImplementedError

    def get_effect_str(self):
        raise NotImplementedError

    def parametrize(self, ff_line):
        self.parametrized = True
        return True

    def is_parametrized(self):
        return self.parametrized

    def reset_parametrization(self):
        print 'resetting parametrization'
        self.parametrized = False
        return False

    def __str__(self):
        return self.get_name_str()

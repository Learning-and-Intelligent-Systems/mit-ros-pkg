#!/usr/bin/env python
import roslib
roslib.load_manifest('smachforward')
from fstate import *
from predicate_viewer import *
import ff_utils
import smach

def aggregate_keys(sub_fstates, predicate_estimator):
    all_in_keys = list()
    all_out_keys = list()
    all_in_keys.extend(predicate_estimator.get_in_keys())
    for sub_fstate in sub_fstates:
        all_in_keys.extend(sub_fstate.get_input_keys())
        all_in_keys.extend(sub_fstate.get_output_keys())
        all_out_keys.extend(sub_fstate.get_output_keys())
        all_out_keys.extend(sub_fstate.get_input_keys())
    print 'all in keys:', all_in_keys
    unique_in_keys = set(all_in_keys)
    unique_out_keys = set(all_out_keys)
    in_keys = list(unique_in_keys)
    in_keys.sort()
    out_keys = list(unique_out_keys)
    out_keys.sort()
    return in_keys, out_keys

class PredicateViewerFState(FState):
    def __init__(self, in_keys, out_keys, predicate_viewer):
        FState.__init__(self, in_keys, out_keys)
        self.predicate_viewer = predicate_viewer

    def execute(self, userdata):
        rospy.loginfo('Executing state PredicateViewerFState')
        raw_input('\n\n\npress enter to continue...')
        self.predicate_viewer.view_predicates(userdata)
        entry = raw_input('\n\n\npress enter to exit and continue (or f to return failure)...')
        if entry == 'f':
            return 'failure'
        return 'success'

class FStateMachine(FState):
    def __init__(self, name, sub_fstates, predicate_estimator, goal, max_recursion_depth = 5, view_predicates=False):
        in_keys, out_keys = aggregate_keys(sub_fstates, predicate_estimator)
        FState.__init__(self, in_keys, out_keys)
        self.goal = goal
        self.predicate_estimator = predicate_estimator
        self.sub_fstates = sub_fstates
        self.domain_str, predicates = ff_utils.generate_domain_str(name, self.sub_fstates)
        self.predicates = self.predicate_estimator.rename_and_expand_predicate_list(predicates)
        # self.predicates is the list of all predicates that are check by the set
        # of sub_fstates.  these predicates are renamed to be like (reset ?rarm) instead
        # of (reset ?a).
        self.max_recursion_depth = max_recursion_depth
        self.recursion_depth = 0
        self.pv = PredicateViewer()
        self.pv.add_estimator(self.predicate_estimator, self.predicates)
        self.used_state_names = list()
        self.name = name
        self.view_predicates=view_predicates

    def get_name_str(self):
        return self.name
    
    def execute(self, userdata):
        print 'executing fstate machine...'
        self.pv.view_predicates(userdata)
        # generates the problem string by evaluating all of the (already renamed) predicates
        problem_str = self.predicate_estimator.generate_problem_str(userdata, self.predicates, self.goal)
        ff_output = ff_utils.execute_ff(self.domain_str, problem_str)
        if ff_output == None:
            print 'COULD NOT FIND A LEGAL PLAN'
            return 'failure'
        for substate in self.sub_fstates:
            substate.reset_parametrization()
        paramed_fstate_lst = ff_utils.generate_parametrized_fstate_lst(self.sub_fstates, ff_output)
        if paramed_fstate_lst == None:
            print 'COULD NOT PARAMETRIZE PLAN'
            return 'failure'
        sm = smach.StateMachine(outcomes = ['success', 'nested_fsm_failure'], input_keys = self.in_keys, output_keys = self.out_keys)
        sm.userdata = userdata  #NOTE: this may fuck things up
        fail_state = 'nested_fsm_failure'
        state_list = list()
        with sm:
            for i, fstate in enumerate(paramed_fstate_lst):
                fstate_name = fstate.get_name_str()
                if fstate_name in self.used_state_names:
                    fstate_name = fstate_name + str(i)
                self.used_state_names.append(fstate_name)
                if i + 1 >= len(paramed_fstate_lst):
                    success = 'success'
                else:
                    success_name = paramed_fstate_lst[i+1].get_name_str()
                    if success_name in self.used_state_names:
                        success_name = success_name + str(i+1)
                    success = success_name
                if self.view_predicates:
                    predicate_viewer_state = PredicateViewerFState(self.in_keys, self.out_keys, self.pv)
                    predicate_viewer_name = '\tpredicate_viewer_'+str(i)
                    smach.StateMachine.add(fstate_name, fstate,
                                           transitions={'success':predicate_viewer_name,
                                                        'failure':fail_state})
                    state_list.append(fstate_name)
                    smach.StateMachine.add(predicate_viewer_name, predicate_viewer_state,
                                           transitions={'success':success,
                                                        'failure':fail_state})
                    state_list.append(predicate_viewer_name)
                else:
                    smach.StateMachine.add(fstate_name, fstate,
                                           transitions={'success':success,
                                                        'failure':fail_state})
                    state_list.append(fstate_name)

        print '\n\n\nSTARTING NESTED FSM \n'
        for state in state_list:
            print state
        print '\n\n\n\n'
        raw_input('enter to continue...')
        outcome = sm.execute()
        print 'nested state machine complete'
        if outcome is 'success':
            return 'success'
        else:
            self.recursion_depth = self.recursion_depth + 1
            if self.recursion_depth < self.max_recursion_depth:
                print '\n\n\n\nFINITE STATE MACHINE FAILURE...\n\n\n\n'
                raw_input('press enter to recurse and continue...')
                return self.execute(userdata)
            else:
                return 'failure'

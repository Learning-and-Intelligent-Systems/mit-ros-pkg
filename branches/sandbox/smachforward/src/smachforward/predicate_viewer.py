#!/usr/bin/env python
import roslib
roslib.load_manifest('smachforward')
from predicate_logging import *

class PredicateViewer():
    predicate_viewer_instance = None

    @staticmethod
    def get_predicate_viewer():
        if PredicateViewer.predicate_viewer_instance == None:
            PredicateViewer.predicate_viewer_instance = PredicateViewer()
        return PredicateViewer.predicate_viewer_instance

    def __init__(self):
        self.estimators = list()
    
    def add_estimator(self, predicate_estimator, predicate_lst):
        self.estimators.append((predicate_estimator, predicate_lst))

    def view_predicates(self, userdata):
        for (estimator, predicate_lst) in self.estimators:
            predicate_logger_client = PredicateLoggerClient(estimator.domain)
            predicate_logger_client.refresh()
            print '========================================================='
            print 'domain:', estimator.domain
            print '============================'
            for predicate in predicate_lst:
                lower = predicate.lower()
                if lower == predicate:
                    # this filters out all the upper case ones, like GRIPPER
                    value = estimator.get_predicate_value(userdata, predicate)
                    print predicate,
                    if value:
                        print '\t\tTRUE'
                    else:
                        print ''
                    predicate_logger_client.log(predicate, value) 
                else:
                    value = True
                    predicate_logger_client.log(predicate, value, '.')
                    print predicate, '.'

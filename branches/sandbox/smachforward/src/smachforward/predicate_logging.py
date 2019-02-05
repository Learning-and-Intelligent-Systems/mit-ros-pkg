#!/usr/bin/env python
import roslib
roslib.load_manifest('smachforward')
roslib.load_manifest('bakebot')
import rospy
import time
from smachforward.msg import *
from smachforward.srv import *

class PredicateLogger():
    def __init__(self):
        rospy.loginfo('instantiating a new predicate logger')
        s = rospy.Service('store_predicates', PredicateRequest, self.handle_store_predicate_srv)
        rospy.loginfo('ready to store predicates')
        rospy.Subscriber('predicates', PredicateMessage, self.handle_predicate_msg)
        now = time.time()
        self.last_message_time = now
        self.stored_values = dict()

    def handle_store_predicate_srv(self, req):
        domain = req.domain
        variable = req.variable
        if domain in self.stored_values:
            if variable in self.stored_values[domain]:
                value = str(variable)
                return PredicateRequestResponse(value)
        return PredicateRequestResponse('None')

    def handle_predicate_msg(self, req):
        now = time.time()
        time_since_last = now - self.last_message_time 
        self.last_message_time = now
        if req.msg_type == 'display':
            if time_since_last > 20:
                print '\n(elapsed seconds: '+str(time_since_last)+'\n\n'
            print req.domain + ': ' + req.variable,
            if req.value == 'True':
                print '\tTRUE\t',
            else:
                print '\t\t\t',
            print req.note
        elif req.msg_type == 'break':
            print '\n\n\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n'
        elif req.msg_type == 'store_value':
            print 'storing value: ', req.domain, req.variable, req.value, req.note
            # save this value for now
            if not req.domain in self.stored_values:
                self.stored_values[req.domain] = dict()
            value = (req.value == 'True')
            self.stored_values[req.variable] = value 
        elif req.msg_type == 'clear_stored_values':
            # erase everything that I have
            # TODO: may want to do this domain specific later
            print 'clearing stored values'
            self.stored_values = dict()

class PredicateLoggerClient():
    def __init__(self, domain='none'):
        self.domain = domain
        self.pub = rospy.Publisher('predicates', PredicateMessage)
        print 'waiting for store predicates service...'
        rospy.wait_for_service('store_predicates')
        print 'done'

    def log(self, predicate, value, note=None):
        sendnote = str(note) if note else ''
        sendvalue = str(value) 
        self.pub.publish(PredicateMessage('display', self.domain, predicate, sendvalue, sendnote))

    def get_stored(self, predicate):
        stored_value = rospy.ServiceProxy('store_predicates', PredicateRequest)
        resp = stored_value(self.domain, predicate)
        if resp.value == 'None':
            print 'no stored value found'
            return None
        elif resp.value == 'True':
            return True
        else:
            return False
    
    def store(self, predicate, value, note=None):
        sendvalue = str(value) 
        self.pub.publish(PredicateMessage('store_value', self.domain, predicate, sendvalue, sendnote))

    def clear_stored(self):
        self.pub.publish(PredicateMessage('clear_stored_values', self.domain, '', '', ''))

    def refresh(self):
        self.pub.publish(PredicateMessage('break', '', '', '', ''))

if __name__ == '__main__':
    rospy.init_node('predicate_logger', anonymous=True)
    logger = PredicateLogger()
    rospy.spin()


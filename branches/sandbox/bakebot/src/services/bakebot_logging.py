#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
from bakebot.srv import *
import math
import os
import sys
import time
from datetime import datetime, date, time

class Activity():
    next_id = 0
    def __init__(self, msg):
        self.unique_id = Activity.next_id
        Activity.next_id = Activity.next_id + 1
        self.active = True
        self.written = False
        self.msg = msg

    def write_start(self, openfile, indent_level = 0):
        indent = ''
        for i in range(0, indent_level):
            indent = indent + '\t'
        time = self.msg.event_time.secs
        line = indent + 'START | ' + str(self.msg.event_type) + ' (#' + str(self.unique_id) + ') @ ' + str(time) + ' | ' + str(self.msg.result) + ' | ' + str(self.msg.annotation) + '\n'
        try:
            openfile.write(line)
            print line,
        except Exception as e:
            rospy.logerr('could not write to file')
            print e
            return False
        self.myfile = openfile
        self.indent_level = indent_level
        self.active = True
        self.written = True
        return True
        
    def write_close(self, msg):
        indent = ''
        for i in range(0, self.indent_level):
            indent = indent + '\t'
        time = msg.event_time.secs
        duration = msg.event_time.secs - self.msg.event_time.secs
        line = indent + 'STOP  | ' + str(msg.event_type) + ' (#' + str(self.unique_id) + ') @ ' + str(time) + ' (duration  ' + str(duration) + ')  | ' + str(msg.result) + ' | ' + str(msg.annotation) + '\n'
        try:
            self.myfile.write(line)
            print line,
        except Exception as e:
            rospy.logerr('could not write to file')
            print e
            return False
        self.active = False
        self.written = True
        return True

class EventLoggerClient():

    loggerserver = None

    def __init__(self, event_type, annotation = '', autostart = True):
        self.event_type = event_type
        self.annotation = annotation
        self.stopped = False
        if autostart:
            self.start()

    def start(self):
        rospy.loginfo('waiting for service bakebot_event_logging...')
        self.pub = rospy.ServiceProxy('bakebot_event_logging', BakebotLogEvent)
        try:
            resp = self.pub(self.event_type,EventLogger.event_action_start, '', self.annotation, rospy.Time.now())
            return (resp >= 0)
        except rospy.ServiceException, e:
            print 'exception caught', e
            return False

    def stop(self, result, annotation = ''):
        if self.stopped:
            rospy.logwarn('this logger client is already stopped')
            return False
        else:
            self.stopped = True
            try:
                resp = self.pub(self.event_type, EventLogger.event_action_stop, result, annotation, rospy.Time.now())
                return (resp >= 0)
            except rospy.ServiceException, e:
                print 'exception caught', e
                return False
            return (resp >= 0)

    def stops(self, annotation = ''):
        return self.stop(EventLogger.result_success, annotation)

    def stopf(self, annotation = ''):
        return self.stop(EventLogger.result_failure, annotation)

    @staticmethod
    def startfsm(stateclassname):
        return EventLoggerClient(EventLogger.event_type_fsm_state, annotation = 'FSM stateclass ' + str(stateclassname), autostart = True)

    @staticmethod
    def bpstart(stateclassname):
        return EventLoggerClient(EventLogger.event_type_fsm_state, annotation = 'breakpoint ' + str(stateclassname), autostart = True)

    @staticmethod
    def startlogger(filename = None):
        rospy.loginfo('starting bakebot logging')
        if filename is None:
            t = datetime.now()
            # TODO will want to fix this so that file location is read from .bashrc
            path = os.environ['BAKEBOT_LOG_PATH']
            filename = path+date.today().isoformat() + '_bakebotlogging.log'
            print filename
        EventLoggerClient.loggerserver = EventLoggerClient(EventLogger.event_type_logging_cmd, annotation = filename, autostart = True)
        rospy.loginfo('logging started')
        return EventLoggerClient.loggerserver

    @staticmethod
    def stoplogger(result):
        return EventLoggerClient.loggerserver.stop(result)


class StaticLoggerLocker():
    events = dict()
    
    @staticmethod
    def check_in_logger(unique_name, event_logger_client):
        StaticLoggerLocker.events[unique_name] = event_logger_client
        rospy.loginfo('checking in ' + str(unique_name) + ' with ' + str(event_logger_client))
    
    @staticmethod
    def check_out_logger(unique_name):
        if StaticLoggerLocker.events == None:
            rospy.logwarn('NO EVENTS')
            return None
        elif not StaticLoggerLocker.events.has_key(unique_name):
            rospy.logwarn('the locker does not have that key: ' + unique_name)
            print StaticLoggerLocker.events
        else:
            return StaticLoggerLocker.events.pop(unique_name)

    
class EventLogger():
    event_logger_instance = None

    event_type_logging_cmd = 'logging'
    event_type_fsm_state = 'fsm'
    event_type_move_arm = 'move_arm'
    event_type_mixing = 'mixing'
    event_type_grab = 'grab'
    event_type_milestone = 'milestone'
    event_type_object_detection = 'object_detection'

    event_action_start = 'start'
    event_action_stop = 'stop'

    result_success = 'success'
    result_failure = 'failure'
    result_aborted = 'aborted'
    
    @staticmethod
    def get_event_logger():
        if EventLogger.event_logger_instance is None:
            EventLogger.event_logger_instance = EventLogger()
        return EventLogger.event_logger_instance

    def __init__(self):
        rospy.loginfo('instantiating a new event logger')
        self.write_enabled = False
        self.current_activities = list()

    def start_writing(self, filename):
        rospy.loginfo('starting writing (appending) to: ' + filename)
        try:
            self.myfile = open(filename, 'a')
            t = datetime.now()
            self.myfile.write('\n\n\n' + str(t.isoformat() + '\n\n'))
            self.write_enabled = True
            self.filename = filename
            self.current_activities = list()
        except Exception as e:
            rospy.logerr('opening the file for writing failed')
            print e
            self.myfile = None
            self.write_enabled = False
            self.filename = None
            self.current_activities = list()
            return False
        return True

    def stop_writing(self):
        rospy.loginfo('closing ' + self.filename)
        self.write_enabled = False
        self.filename = None
        if self.myfile is None:
            rospy.logwarn('cannot close a None file')
            return False
        try:
            self.myfile.close()
        except Exception as e:
            rospy.logerr('closing the file failed')
            print e
        return True
    
def handle_log_msg(req):
    if req.event_type == EventLogger.event_type_logging_cmd:
        if req.event_action == EventLogger.event_action_start:
            status = EventLogger.event_logger_instance.start_writing(req.annotation)
            if not status:
                rospy.logerr('something went wrong in the file opening')
                return BakebotLogEventResponse(-100)
    if EventLogger.event_logger_instance.write_enabled:
        if req.event_action == EventLogger.event_action_stop:
            # go through the list and find the one with the right event_type to close
            # close all up to that type
            while len(EventLogger.event_logger_instance.current_activities) is not 0:
                activity = EventLogger.event_logger_instance.current_activities.pop()
                if activity.msg.event_type == req.event_type:
                    activity.write_close(req)
                    break
            if len(EventLogger.event_logger_instance.current_activities) is 0:
                rospy.loginfo('out of activities')
                EventLogger.event_logger_instance.stop_writing()
                if req.event_type == EventLogger.event_type_logging_cmd:
                    # this means that we want to stop logging
                    rospy.loginfo('received stop logging command')
                else:
                    return BakebotLogEventResponse(-2)
            return BakebotLogEventResponse(0)
                
        elif req.event_action == EventLogger.event_action_start:
            newaction = Activity(req)
            newaction.write_start(EventLogger.event_logger_instance.myfile, len(EventLogger.event_logger_instance.current_activities))
            EventLogger.event_logger_instance.current_activities.append(newaction)
            return BakebotLogEventResponse(0)
                

if __name__ == '__main__':
    rospy.init_node('event_logger', anonymous=True)
    s = rospy.Service('bakebot_event_logging', BakebotLogEvent, handle_log_msg)
    try:
        logger = EventLogger.get_event_logger()
        rospy.spin() 
    finally:
        if logger.write_enabled:
            logger.stop_writing()




import roslib; roslib.load_manifest('demo_exec')
import rospy

def askYN(question):
    rospy.loginfo('%s (y/n)', question)
    valid_ans = False
    while not valid_ans:
        try:
            ans = str(raw_input())
            valid_ans = True
            if ans == 'n':
                return False
            if ans != 'y':
                valid_ans = False
        except ValueError:
            valid_ans = False
        if not valid_ans:
            rospy.loginfo('Please enter y or n')
    return True

def pause(question):
    rospy.loginfo('%s (Press any key to continue)', question)
    ans = raw_input()
    return

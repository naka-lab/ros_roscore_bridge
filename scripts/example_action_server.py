#!/usr/bin/env python
from __future__ import print_function, unicode_literals
import rospy
import actionlib
from actionlib.msg import *
import time

def callback( goal ):
    result = goal.goal

    for i in range(10):
        result += 10

        print(result)
        action_server.publish_feedback( TestFeedback(result) )
        time.sleep(1)

    action_server.set_succeeded( TestResult(result) )


def main():
    action_server.start()
    rospy.spin() 
    
    
if __name__ == "__main__":
    rospy.init_node("action_server")
    action_server = actionlib.SimpleActionServer("test_action", TestAction, execute_cb=callback, auto_start=False)

    main()
#!/usr/bin/env python
from __future__ import print_function, unicode_literals
from std_srvs.srv import *
import rospy
import time

def f(req):
    print( "recieve:", req.data )
    time.sleep(3)
    return SetBoolResponse( True, "test" )

def add_two_ints_server():
    rospy.init_node('servise_server')
    s = rospy.Service('set_bool', SetBool, f)
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()

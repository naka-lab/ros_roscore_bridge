#!/usr/bin/env python
from __future__ import print_function, unicode_literals
import rospy
import rospkg
from actionlib.msg import *
import os
import sys

# roscor_bridgeへのパスを追加・import
path = rospkg.RosPack().get_path("ros_roscore_bridge")
sys.path.append( os.path.join( path, "lib" ) )
import roscore_bridge


def main():
    # 接続先を設定
    node_a = roscore_bridge.MPNode( "http://127.0.0.1:11311", "A" )

    print("wait for action:")
    act = node_a.SimpleActionClient( "test_action", TestAction )
    act.wait_for_server()

    print("call action:")
    act.send_goal( TestGoal(100) )

    print("wait for result:")
    act.wait_for_result()

    print("action reult:", act.get_result() )
    


if __name__ == '__main__':
    main()
#!/usr/bin/env python
from __future__ import print_function, unicode_literals
import rospy
import rospkg
from std_srvs.srv import *
import os
import sys

# roscor_bridgeへのパスを追加・import
path = rospkg.RosPack().get_path("ros_roscore_bridge")
sys.path.append( os.path.join( path, "lib" ) )
import roscore_bridge


def main():
    # 接続先を設定
    node_a = roscore_bridge.MPNode( "http://127.0.0.1:11311", "A" )

    print("wait for srv:")
    node_a.wait_for_service( "set_bool" )

    print("call srv:")
    srv = node_a.ServiceProxy( "set_bool", SetBool )
    print("serv responce:", srv(True))
    


if __name__ == '__main__':
    main()
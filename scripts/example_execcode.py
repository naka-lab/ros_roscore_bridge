#!/usr/bin/env python
from __future__ import print_function, unicode_literals
import rospy
import rospkg
from std_msgs.msg import String
import os
import sys
import time

# roscor_bridgeへのパスを追加・import
path = rospkg.RosPack().get_path("ros_roscore_bridge")
sys.path.append( os.path.join( path, "lib" ) )
import roscore_bridge

code = """
import time
import rospy

pub = rospy.Publisher('chatter', String, queue_size=10)
for i in range(100):
    str = "hello world %d" % i
    pub.publish(str)
    time.sleep(0.1)
"""

def main():
    # 接続先を設定
    node_a = roscore_bridge.MPNode( "http://127.0.0.1:11311", "A" )
    node_b = roscore_bridge.MPNode( "http://127.0.0.1:11312", "B" )

    # 任意のコードを実行（複数行，非同期実行）
    print("実行1")
    node_a.exec( code, sync=False ) # sync=Falseで非同期実行
    print("終了1")

    # 任意のコードを実行（1行ずつ，同期実行）
    print("実行2")
    node_b.exec( "import time" )
    node_b.exec( "import rospy" )
    node_b.exec( "pub = rospy.Publisher('chatter', String, queue_size=10)" )
    for i in range(100):
        node_b.exec( "str = 'hello world %d'" % i)
        node_b.exec( "pub.publish(str)" )
        time.sleep(0.1)
    print("終了2")

if __name__ == '__main__':
    main()
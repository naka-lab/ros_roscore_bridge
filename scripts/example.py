#!/usr/bin/env python
from __future__ import print_function, unicode_literals
import rospy
import rospkg
from std_msgs.msg import String
import os
import sys

# roscor_bridgeへのパスを追加・import
path = rospkg.RosPack().get_path("ros_roscore_bridge")
sys.path.append( os.path.join( path, "lib" ) )
import roscore_bridge

def callback( data ):
    print( "callback", data )

def main():
    # 接続先を設定
    node_a = roscore_bridge.MPNode( "http://192.168.1.14:11311", "A" )
    node_b = roscore_bridge.MPNode( "http://192.168.1.74:11311", "B" )

    # SubscriberとPublisherを用意
    sub_a = node_a.Subscriber("chatter", String, 1, callback )
    sub_b = node_b.Subscriber("chatter", String, 1 )

    pub_a = node_a.Publisher("chatter2", String )

    # ノードを開始
    node_a.start_node()
    node_b.start_node()

    while not rospy.is_shutdown():
        # キューからデータを取り出す
        data = sub_b.get_message()
        print("main：", data)

        # Bが受信したデータをAに転送
        pub_a.publish( data )


if __name__ == '__main__':
    main()
#!/usr/bin/env python
from __future__ import print_function, unicode_literals
import rospy
from std_msgs.msg import String
import time
import multiprocessing
import os
import queue
import threading

class MPPublisherST():
    def __init__(self, node, name, type_):
        self.node = node
        self.topic_name = name
        self.topic_type = type_

    def publish(self, data):
        self.node.publish( self, data )

class MPSubscriberST():
    def __init__(self, node, name, type_, que_size ):
        self.node = node
        self.topic_name = name
        self.topic_type = type_
        self.que_size = que_size
        self.queue = multiprocessing.Queue()

    def callback(self, data):
        self.queue.put( data )

        while self.queue.qsize()>self.que_size:
            try:
                self.queue.get()
            except queue.Empty:
                pass

    def get_message(self, timeout=None):
        return self.queue.get(timeout=timeout)

def call_user_callback( user_callback, sub ):
    while not rospy.is_shutdown():
        # キューにデータがあれば実行
        try:
            data = sub.get_message(timeout=1)
        except queue.Empty:
            continue

        user_callback( data )

class MPNodeST():
    def __init__( self, master_uri, node_name ):
        self.queue_pub = multiprocessing.Queue()

        self.node_name = node_name
        self.master_uri = master_uri

        self.subscribers = []
        self.publishers = []


    def Subscriber( self, name, type_, que_size=10, callback=None, **kwargs ):
        sub = MPSubscriberST( self, name, type_, que_size )
        self.subscribers.append( (sub, kwargs) )

        if callback:
            t = threading.Thread( target=call_user_callback, args=(callback, sub) )
            t.start()
        return sub

    def Publisher( self, name, type_, **kwargs ):
        pub = MPPublisherST( self, name, type_ )
        self.publishers.append( (pub, kwargs) )
        return pub
    
    def publish( self, pub, data ):
        self.queue_pub.put( (pub.topic_name, data) )

    def shutdown(self):
        self.queue_pub.put((None, None))

    def start_node(self):
        p = multiprocessing.Process(target=self.run )
        p.start()

    def run( self ):
        print(self.node_name, "開始")

        # ros_master_uriを書き換えてnodeを実行
        os.environ['ROS_MASTER_URI'] = self.master_uri
        print(os.environ['ROS_MASTER_URI'])
        rospy.init_node( self.node_name )

        # subsciber
        for sub in self.subscribers:
            rospy.Subscriber( sub[0].topic_name, sub[0].topic_type, sub[0].callback, **sub[1] )

        # publisher 
        topicname2publisher = {}
        for pub in self.publishers:
            p = rospy.Publisher( pub[0].topic_name , pub[0].topic_type, queue_size=10, **pub[1])
            topicname2publisher[pub[0].topic_name] = p

        while not rospy.is_shutdown():
            # キューにデータがあれば実行
            try:
                topic_name, data = self.queue_pub.get(timeout=1)
            except queue.Empty:
                continue

            # shutdown
            if topic_name==None and data==None:
                rospy.signal_shutdown("MPNode.shudown() is called. ")
                break
            topicname2publisher[topic_name].publish( data )

def callback( data ):
    print( "callback", data )

def main():
    # 接続先を設定
    node_a = MPNodeST( "http://127.0.0.1:11311", "A" )
    node_b = MPNodeST( "http://127.0.0.1:11312", "B" )

    # SubscriberとPublisherを用意
    sub_a = node_a.Subscriber("chatter", String, 1, callback )
    sub_b = node_b.Subscriber("chatter", String, 1 )

    pub_a = node_a.Publisher("chatter2", String, latch=True )

    # ノードを開始
    node_a.start_node()
    node_b.start_node()

    while not rospy.is_shutdown():
        # キューからデータを取り出す
        data = sub_b.get_message()

        print( data, "を転送" )
        pub_a.publish( data )

if __name__ == '__main__':
    main()

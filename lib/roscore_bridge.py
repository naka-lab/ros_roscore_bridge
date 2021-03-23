#!/usr/bin/env python
from __future__ import print_function, unicode_literals
import rospy
from std_msgs.msg import String
import time
import multiprocessing
import multiprocessing.queues
import os
import queue
import threading
import time


CMD_PUB_NEW = 0
CMD_PUB_PUBLISH = 1

CMD_SUB_NEW = 100
CMD_SUB_SUBSCRIBE = 101
CMD_SUB_UNREGISTER = 102

CMD_NODE_SHUDDOWN = 200

id_counter = 0


class MPPublisher():
    def __init__(self, node, name, type_):
        global id_counter
        self.node = node
        self.topic_name = name
        self.topic_type = type_
        self.id = id_counter

        id_counter += 1

    def publish(self, data):
        #self.node.publish( self, data )
        self.node.put_cmd_queue( self.id, CMD_PUB_PUBLISH, {"data":data} )

class MPSubscriber():
    def __init__(self, node, name, type_, que_size ):
        global id_counter

        self.node = node
        self.topic_name = name
        self.topic_type = type_
        self.que_size = que_size
        self.queue = multiprocessing.Queue()
        self.id = id_counter
        id_counter += 1

    def put_message(self, data):
        self.queue.put( data )
        while self.queue.qsize()>self.que_size:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass

    def get_message(self, timeout=None):
        return self.queue.get(timeout=timeout)

    def unregister(self):
        self.node.put_cmd_queue( self.id, CMD_SUB_UNREGISTER, {} )


class MPNode():
    def __init__( self, master_uri, node_name ):
        self.queue_cmd = multiprocessing.Queue()
        self.queue_sub = multiprocessing.Queue()

        self.node_name = node_name
        self.master_uri = master_uri

        self.subscribers = {}
        self.mp_subscribers = {}
        self.publishers = {}

        t = threading.Thread( target=self.th_transfer_subscribed_data )
        t.setDaemon(True)
        t.start()

        p = multiprocessing.Process(target=self.run )
        p.start()

    # 各Subscriberが受信したデータをメインプロセスと共有するcallbac（nodeプロセスで実行）
    def cb_data_recieved(self, data, args):
        sub_id = args[0] 
        #print("キュー追加", self.queue_sub.qsize())
        self.queue_sub.put( (sub_id, data) )

    # 受信したデータを各MPSubscriberへ転送するスレッド（メインプロセスで実行）
    def th_transfer_subscribed_data(self):
        while not rospy.is_shutdown():
            try:
                sub_id, data = self.queue_sub.get(timeout=1)
            except queue.Empty:
                continue

            if sub_id in self.mp_subscribers:
                #print("データ転送", sub_id)
                self.mp_subscribers[sub_id].put_message( data )
            else:
                print("th_transfer_subscribed_data: id does not exits. ")

    # ユーザが設定したcallbackを呼ぶスレッド（メインプロセスで実行）
    def th_call_user_callback( self,  user_callback, sub ):
        while not rospy.is_shutdown():
            # キューにデータがあれば実行
            try:
                data = sub.get_message(timeout=1)
            except queue.Empty:
                continue

            #print("callback")
            user_callback( data )

    def Subscriber( self, name, type_, que_size=10, callback=None ):
        sub = MPSubscriber( self, name, type_, que_size )
        self.mp_subscribers[sub.id] = sub

        if callback:
            t = threading.Thread( target=self.th_call_user_callback, args=(callback, sub) )
            t.setDaemon(True)
            t.start()
        
        args_dict = { "name":sub.topic_name, "data_class":sub.topic_type }
        self.put_cmd_queue( sub.id, CMD_SUB_NEW, args_dict)
        return sub

    def Publisher( self, name, type_, queue_size=10 ):
        pub = MPPublisher( self, name, type_ )

        args_dict = { "name":pub.topic_name,  "data_class":pub.topic_type , "queue_size":queue_size}
        self.put_cmd_queue( pub.id, CMD_PUB_NEW, args_dict )

        return pub
    
    #def start_node(self):
    #    p = multiprocessing.Process(target=self.run )
    #    p.start()

    def shutdown(self):
        self.put_cmd_queue( -1, CMD_NODE_SHUDDOWN )
        

    def put_cmd_queue(self, sender_id, cmd, args_dict=None):
        self.queue_cmd.put( (sender_id, cmd, args_dict) )

    def run( self ):
        print(self.node_name, "開始")

        # ros_master_uriを書き換えてnodeを実行
        os.environ['ROS_MASTER_URI'] = self.master_uri
        print(os.environ['ROS_MASTER_URI'])
        rospy.init_node( self.node_name )

        while not rospy.is_shutdown():
            # キューにデータがあれば実行
            try:
                sender_id, cmd, args_dict = self.queue_cmd.get(timeout=1)
            except queue.Empty:
                continue

            if cmd==CMD_SUB_NEW:
                print("new subscribe: ", sender_id, args_dict )
                args_dict["callback"] = self.cb_data_recieved
                args_dict["callback_args"] = (sender_id,)
                sub = rospy.Subscriber( **args_dict )
                self.subscribers[sender_id] = sub
            elif cmd==CMD_PUB_NEW:
                print("new publisher: ", sender_id, args_dict )
                pub = rospy.Publisher( **args_dict )
                self.subscribers[sender_id] = pub
            elif cmd==CMD_SUB_UNREGISTER:
                self.subscribers[sender_id].unregister()
            elif cmd==CMD_PUB_PUBLISH:
                #print("publish: ", sender_id, args_dict )
                self.subscribers[sender_id].publish( args_dict["data"] )
            elif cmd==CMD_NODE_SHUDDOWN:
                print("shutdown")
                rospy.signal_shutdown("MPNode.shudown() is called. ")



def callback( data ):
    #print( "callback", data )
    pass

def main():
    # 接続先を設定
    node_a = MPNode( "http://127.0.0.1:11311", "A" )
    node_b = MPNode( "http://127.0.0.1:11312", "B" )


    # SubscriberとPublisherを用意
    sub_a = node_a.Subscriber("chatter", String, 1, callback )
    sub_b = node_b.Subscriber("chatter", String, 1 )

    pub_a = node_a.Publisher("chatter2", String )

    # 動的にノードの設定が変えられるかテスト
    for i in range(5):
        print("unregister")
        sub_b.unregister()
        time.sleep(2)
        print("subscribe")
        sub_b = node_b.Subscriber("chatter", String, 1 )
        time.sleep(2)
        print("shutdown")
        node_a.shutdown()
        time.sleep(2)
        node_a = MPNode( "http://127.0.0.1:11311", "A" )
        time.sleep(2)

        pub_a = node_a.Publisher("chatter2", String )
        time.sleep(2)


    while not rospy.is_shutdown():
        # キューからデータを取り出す
        data = sub_b.get_message()

        print("受信：", data)

        # Bが受信したデータをAに転送
        pub_a.publish( data )


if __name__ == '__main__':
    main()

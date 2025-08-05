#!/usr/bin/env python
from __future__ import print_function, unicode_literals
import time
import multiprocessing
import multiprocessing.queues
import os
import sys
import queue
import threading
import time
from functools import partial


CMD_PUB_NEW = 0
CMD_PUB_PUBLISH = 1

CMD_SUB_NEW = 100
CMD_SUB_SUBSCRIBE = 101
CMD_SUB_UNREGISTER = 102

CMD_SRV_NEW = 200
CMD_SRV_CALL = 201
CMD_SRV_WAITFORSERVICE = 202

CMD_ACT_NEW = 300
CMD_ACT_SENDGOAL = 301
CMD_ACT_WAITFORSERVER = 302
CMD_ACT_WAITFORRESULT = 303
CMD_ACT_GETRESULT = 304

CMD_EXEC_CODE = 400
CMD_NODE_SHUDDOWN = 1000


ROS1_PATH=None
def set_ros1_path( path ):
    global ROS1_PATH
    ROS1_PATH = path

class MPNodeClient(object):
    id_counter = 0
    def __init__(self, node, que_size=0 ):
        self.node = node
        self.queue = queue.Queue()
        self.que_size = que_size

        self.id = MPNodeClient.id_counter
        MPNodeClient.id_counter += 1

    def _put_message(self, data):
        self.queue.put( data )
        while self.queue.qsize()>self.que_size:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass

    def _get_message(self, timeout=None):
        return self.queue.get(timeout=timeout)


class MPPublisher(MPNodeClient):
    def __init__(self, node, name, type_):
        super(MPPublisher, self).__init__(node)
        self.topic_name = name
        self.topic_type = type_

    def publish(self, data):
        self.node.put_cmd_queue( self.id, CMD_PUB_PUBLISH, {"data":data} )

    def __str__(self):
        return f"MPPublisher: {self.node.node_name}.{self.topic_name}"


class MPSubscriber(MPNodeClient):
    def __init__(self, node, name, type_, que_size ):
        super(MPSubscriber, self).__init__(node, que_size)
        self.topic_name = name
        self.topic_type = type_

    def get_message(self, timeout=None):
        return self._get_message( timeout )

    def unregister(self):
        self.node.put_cmd_queue( self.id, CMD_SUB_UNREGISTER, {} )

    def __str__(self):
        return f"MPSubscriber: {self.node.node_name}.{self.topic_name}"


class MPServiceProxy(MPNodeClient):
    def __init__(self, node, name, type_ ):
        super(MPServiceProxy, self).__init__(node, 1)
        self.service_name = name
        self.service_type = type_

    def __call__(self, *args ):
        self.node.put_cmd_queue( self.id, CMD_SRV_CALL, {"args":args} )
        return self._get_message()

    def __str__(self):
        return f"MPServiceProxy: {self.node.node_name}.{self.service_name}"

class MPActionClient(MPNodeClient):
    def __init__(self, node, action_name="" ):
        super(MPActionClient, self).__init__(node, 1)
        self.action_name = action_name
    def wait_for_server(self):
        self.node.put_cmd_queue( self.id, CMD_ACT_WAITFORSERVER )
        return self._get_message()

    def send_goal(self, goal ):
        self.node.put_cmd_queue( self.id, CMD_ACT_SENDGOAL, {"goal":goal} )

    def wait_for_result(self):
        self.node.put_cmd_queue( self.id, CMD_ACT_WAITFORRESULT )
        self.result = self._get_message()

    def get_result(self):
        self.node.put_cmd_queue( self.id, CMD_ACT_GETRESULT )
        return self._get_message()

    def __str__(self):
        return f"MPActionClient: {self.node.node_name}.{self.action_name}"


class MPNode():
    def __init__( self, master_uri, node_name, ros_ip=None, host_name=None ):
        self.queue_from_client = multiprocessing.Queue()
        self.queue_to_client = multiprocessing.Queue()

        self.node_name = node_name
        self.master_uri = master_uri
        self.ros_ip = ros_ip
        self.host_name = host_name

        self.mp_node_clients = {}
        self.publishers = {}

        self.queue_codes = queue.Queue()
        self.is_runnning = True

        t = threading.Thread( target=self.th_transfer_subscribed_data )
        t.setDaemon(True)
        t.start()

        p = multiprocessing.Process(target=self.run )
        p.start()

    # 各Subscriberが受信したデータをメインプロセスと共有するcallbac（nodeプロセスで実行）
    def cb_data_recieved(self, data, args):
        sub_id = args[0] 
        #print("キュー追加", self.queue_sub.qsize())
        self.queue_to_client.put( (sub_id, data) )

    # 受信したデータを各MPSubscriberへ転送するスレッド（メインプロセスで実行）
    def th_transfer_subscribed_data(self):
        while self.is_runnning:
            try:
                sub_id, data = self.queue_to_client.get(timeout=1)
            except queue.Empty:
                continue

            if sub_id in self.mp_node_clients:
                #print("データ転送", sub_id)
                self.mp_node_clients[sub_id]._put_message( data )
            else:
                print("th_transfer_subscribed_data: id does not exits. ")

    # ユーザが設定したcallbackを呼ぶスレッド（メインプロセスで実行）
    def th_call_user_callback( self,  user_callback, sub ):
        while self.is_runnning:
            # キューにデータがあれば実行
            try:
                data = sub.get_message(timeout=1)
            except queue.Empty:
                continue

            #print("callback")
            user_callback( data )

    # 任意のコードを実行するスレッド（クライアントプロセスで実行）
    def th_run_code( self ):
        while self.is_runnning:
            # キューにデータがあれば実行
            try:
                sender_id, exec_code, eval_code = self.queue_codes.get(timeout=1)
            except queue.Empty:
                continue

            # 実行
            retval = None
            if exec_code:
                exec( exec_code )
            
            if eval_code:
                retval = eval(eval_code)

            # 実行し終わったらClientへ通知
            self.queue_to_client.put( (sender_id, retval) )

    # 任意のコードを実行
    def exec( self, code, sync=True ):
        exec = MPNodeClient( self, 1 )
        self.mp_node_clients[exec.id] = exec
        self.put_cmd_queue( exec.id, CMD_EXEC_CODE, {"exec_code":code, "eval_code":None} )

        if sync:
            # 実行が終了するまで待機
            exec._get_message()

    def eval( self, code ):
        eval = MPNodeClient( self, 1 )
        self.mp_node_clients[eval.id] = eval
        self.put_cmd_queue( eval.id, CMD_EXEC_CODE, {"exec_code":None, "eval_code":code} )
        return eval._get_message()

    def Subscriber( self, name, type_, que_size=10, callback=None, **kwargs ):
        sub = MPSubscriber( self, name, type_, que_size )
        self.mp_node_clients[sub.id] = sub

        if callback:
            t = threading.Thread( target=self.th_call_user_callback, args=(callback, sub) )
            t.setDaemon(True)
            t.start()

        args_dict = {}
        args_dict.update( kwargs )
        args_dict.update( { "name":sub.topic_name, "data_class":sub.topic_type } )
        self.put_cmd_queue( sub.id, CMD_SUB_NEW, args_dict)
        return sub

    def Publisher( self, name, type_, queue_size=10, **kwargs ):
        pub = MPPublisher( self, name, type_ )

        args_dict = {}
        args_dict.update( kwargs )
        args_dict.update( { "name":pub.topic_name,  "data_class":pub.topic_type , "queue_size":queue_size} )
        self.put_cmd_queue( pub.id, CMD_PUB_NEW, args_dict )

        return pub

    def ServiceProxy(self, name, service_class):
        srv = MPServiceProxy( self, name, service_class )
        self.mp_node_clients[srv.id] = srv
        self.put_cmd_queue( srv.id, CMD_SRV_NEW, {"name":name, "service_class":service_class})
        return srv

    def wait_for_service(self, name, timeout=None):
        wait = MPNodeClient( self, 1 )
        self.mp_node_clients[wait.id] = wait
        self.put_cmd_queue( wait.id, CMD_SRV_WAITFORSERVICE, {"service":name} )

        wait._get_message( timeout )

    def SimpleActionClient(self, name, action_class):
        act = MPActionClient( self, name )
        self.mp_node_clients[act.id] = act
        self.put_cmd_queue( act.id, CMD_ACT_NEW, {"ns":name, "ActionSpec":action_class})
        return act

    def shutdown(self):
        self.put_cmd_queue( -1, CMD_NODE_SHUDDOWN )
        
    def put_cmd_queue(self, sender_id, cmd, args_dict=None):
        self.queue_from_client.put( (sender_id, cmd, args_dict) )

    def async_call_and_put_retval( self, sender_id,  func ):
        def _thread_func( queue, sender_id ,func ):
            try:
                queue.put( (sender_id, func()) )
            except Exception as e:
                print("\033[31m Error in calling function:", type(e), e, "\033[0m")
                queue.put( (sender_id, None) )
        t = threading.Thread( target=_thread_func, args=(self.queue_to_client, sender_id, func) )
        t.setDaemon(True)
        t.start()

    def run( self ):
        # ROS1のパスを追加
        if ROS1_PATH is not None:
            sys.path.insert(0, ROS1_PATH)

        # 別プロセスでimportすることで，メインプロセスへの影響を最小限に抑える
        import rospy
        import roslib
        import actionlib

        print(self.node_name, "開始")

        ros_objects = {}

        # ros_master_uriを書き換えてnodeを実行
        os.environ['ROS_MASTER_URI'] = self.master_uri

        if self.host_name!=None:
            os.environ['ROS_HOSTNAME'] = self.host_name
        if self.ros_ip!=None:
            os.environ['ROS_IP'] = self.ros_ip
        print(self.node_name, ": ROS_MASTER_URI->", os.environ['ROS_MASTER_URI'])
        print(self.node_name, ": ROS_HOSTNAME->", os.environ['ROS_HOSTNAME'] if "ROS_HOSTNAME" in os.environ else "NONE" )
        print(self.node_name, ": ROS_IP->", os.environ['ROS_IP'] if "ROS_IP" in os.environ else "NONE" )
        rospy.init_node( self.node_name )

        t = threading.Thread( target=self.th_run_code )
        t.setDaemon(True)
        t.start()

        while not rospy.is_shutdown():
            # キューにデータがあれば実行
            try:
                sender_id, cmd, args_dict = self.queue_from_client.get(timeout=1)
            except queue.Empty:
                continue

            if cmd==CMD_SUB_NEW:
                args_dict["callback"] = self.cb_data_recieved
                args_dict["callback_args"] = (sender_id,)
                sub = rospy.Subscriber( **args_dict )
                ros_objects[sender_id] = sub
            elif cmd==CMD_PUB_NEW:
                pub = rospy.Publisher( **args_dict )
                ros_objects[sender_id] = pub
            elif cmd==CMD_SUB_UNREGISTER:
                ros_objects[sender_id].unregister()
            elif cmd==CMD_PUB_PUBLISH:
                #print("publish: ", sender_id, args_dict )
                ros_objects[sender_id].publish( args_dict["data"] )
            elif cmd==CMD_SRV_NEW:
                ros_objects[sender_id] = rospy.ServiceProxy( **args_dict )
            elif cmd==CMD_SRV_CALL:
                self.async_call_and_put_retval( sender_id, partial( ros_objects[sender_id], *args_dict["args"] ) )
            elif cmd==CMD_SRV_WAITFORSERVICE:
                self.async_call_and_put_retval( sender_id, partial( rospy.wait_for_service, **args_dict ) )
            elif cmd==CMD_ACT_NEW:
                act = actionlib.SimpleActionClient( **args_dict )                
                ros_objects[sender_id] = act
            elif cmd==CMD_ACT_SENDGOAL:
                ros_objects[sender_id].send_goal( **args_dict )
            elif cmd==CMD_ACT_WAITFORSERVER:
                self.async_call_and_put_retval( sender_id, ros_objects[sender_id].wait_for_server )
            elif cmd==CMD_ACT_WAITFORRESULT:
                self.async_call_and_put_retval( sender_id, ros_objects[sender_id].wait_for_result )
            elif cmd==CMD_ACT_GETRESULT:
                self.async_call_and_put_retval( sender_id, ros_objects[sender_id].get_result )
            elif cmd==CMD_EXEC_CODE:
                self.queue_codes.put( (sender_id, args_dict["exec_code"], args_dict["eval_code"]) )
            elif cmd==CMD_NODE_SHUDDOWN:
                print("shutdown")
                rospy.signal_shutdown("MPNode.shudown() is called. ")

        self.is_runnning = False



def callback( data ):
    #print( "callback", data )
    pass



def main():
    from std_srvs.srv import String
 
    # 接続先を設定
    node_a = MPNode( "http://127.0.0.1:11311", "A" )
    node_b = MPNode( "http://127.0.0.1:11312", "B" )


    # SubscriberとPublisherを用意
    sub_a = node_a.Subscriber("chatter", String, 1, callback )
    sub_b = node_b.Subscriber("chatter", String, 1 )

    pub_a = node_a.Publisher("chatter2", String, latch=True )

    # 動的にノードの設定が変えられるかテスト
    for i in range(1):
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

    # Serviceのテスト
    print("wait for srv")
    node_b.wait_for_service( "set_bool" )
    srv = node_b.ServiceProxy( "set_bool", SetBool )
    print("serv responce:", srv(True))

    while not rospy.is_shutdown():
        # キューからデータを取り出す
        data = sub_b.get_message()

        print("受信：", data)

        # Bが受信したデータをAに転送
        pub_a.publish( data )


if __name__ == '__main__':
    main()

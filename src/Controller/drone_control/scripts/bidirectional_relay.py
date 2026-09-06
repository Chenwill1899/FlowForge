#!/usr/bin/env python3
"""
双向 topic relay 节点，防止循环传播
防止 /control <-> /drone_X_control 之间的消息循环
"""

import rospy
import threading
import hashlib
import pickle
from controller_msgs.msg import cmd

class BidirectionalRelay:
    def __init__(self):
        rospy.init_node('bidirectional_control_relay', anonymous=True)
        
        # 获取参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.control_topic = rospy.get_param('~control_topic', '/control')
        self.drone_control_topic = rospy.get_param('~drone_control_topic', '/drone_{}_control'.format(self.drone_id))
        
        # 锁用于线程安全
        self.lock = threading.Lock()
        
        # 跟踪最近 relay 的消息哈希值，防止循环（保存最近10条）
        self.recent_relayed_hashes = set()
        self.max_recent_hashes = 10
        
        # 创建发布者
        self.pub_control = rospy.Publisher(self.control_topic, cmd, queue_size=10)
        self.pub_drone_control = rospy.Publisher(self.drone_control_topic, cmd, queue_size=10)
        
        # 创建订阅者（使用队列大小1，避免延迟）
        self.sub_control = rospy.Subscriber(self.control_topic, cmd, self.control_callback, queue_size=1)
        self.sub_drone_control = rospy.Subscriber(self.drone_control_topic, cmd, self.drone_control_callback, queue_size=1)
        
        rospy.loginfo("Bidirectional relay node started")
        rospy.loginfo("  {} <-> {}".format(self.control_topic, self.drone_control_topic))
    
    def _get_message_hash(self, msg):
        """计算消息的哈希值，用于检测重复消息"""
        # 使用时间戳、序列号和命令值来生成哈希
        msg_data = (msg.header.stamp.secs, msg.header.stamp.nsecs, 
                   msg.header.seq, msg.cmd)
        return hashlib.md5(pickle.dumps(msg_data)).hexdigest()
    
    def control_callback(self, msg):
        """处理来自 /control 的消息"""
        msg_hash = self._get_message_hash(msg)
        
        with self.lock:
            # 如果这个消息最近被 relay 过，忽略它（防止循环）
            if msg_hash in self.recent_relayed_hashes:
                return
            
            # 添加到已 relay 的哈希集合
            self.recent_relayed_hashes.add(msg_hash)
            # 限制集合大小
            if len(self.recent_relayed_hashes) > self.max_recent_hashes:
                # 移除最旧的一个（简单方法：清空并重新开始）
                if len(self.recent_relayed_hashes) > self.max_recent_hashes * 2:
                    self.recent_relayed_hashes.clear()
                    self.recent_relayed_hashes.add(msg_hash)
        
        # 发布到 drone_control topic
        self.pub_drone_control.publish(msg)
    
    def drone_control_callback(self, msg):
        """处理来自 /drone_X_control 的消息"""
        msg_hash = self._get_message_hash(msg)
        
        with self.lock:
            # 如果这个消息最近被 relay 过，忽略它（防止循环）
            if msg_hash in self.recent_relayed_hashes:
                return
            
            # 添加到已 relay 的哈希集合
            self.recent_relayed_hashes.add(msg_hash)
            # 限制集合大小
            if len(self.recent_relayed_hashes) > self.max_recent_hashes:
                # 移除最旧的一个（简单方法：清空并重新开始）
                if len(self.recent_relayed_hashes) > self.max_recent_hashes * 2:
                    self.recent_relayed_hashes.clear()
                    self.recent_relayed_hashes.add(msg_hash)
        
        # 发布到 control topic
        self.pub_control.publish(msg)

if __name__ == '__main__':
    try:
        relay = BidirectionalRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


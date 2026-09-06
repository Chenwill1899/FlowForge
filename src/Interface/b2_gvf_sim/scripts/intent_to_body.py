#!/usr/bin/env python3
"""Republish a world-frame navigation intent as a body-frame stick display."""

import math

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class IntentToBody:
    def __init__(self):
        self.yaw = None
        self.output_frame = rospy.get_param("~output_frame", "base")
        odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        intent_topic = rospy.get_param("~intent_topic", "/human_intent")
        output_topic = rospy.get_param("~output_topic", "/b2/human_intent_stick")
        self.publisher = rospy.Publisher(output_topic, TwistStamped, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(intent_topic, TwistStamped, self.intent_callback, queue_size=1)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def intent_callback(self, msg):
        if self.yaw is None:
            return
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        body = TwistStamped()
        body.header = msg.header
        body.header.frame_id = self.output_frame
        body.twist.linear.x = cos_yaw * msg.twist.linear.x + sin_yaw * msg.twist.linear.y
        body.twist.linear.y = -sin_yaw * msg.twist.linear.x + cos_yaw * msg.twist.linear.y
        body.twist.linear.z = msg.twist.linear.z
        body.twist.angular = msg.twist.angular
        self.publisher.publish(body)


if __name__ == "__main__":
    rospy.init_node("intent_to_body")
    IntentToBody()
    rospy.spin()

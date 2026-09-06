#!/usr/bin/env python3
"""Clip visualization point clouds below an optional fixed or odometry height."""

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class HeightClipPointCloud:
    def __init__(self):
        self.height = None
        self.min_z = rospy.get_param("~min_z", None)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/Odometry"),
                         Odometry, self.odom_callback, queue_size=1)
        for topic in rospy.get_param("~topics"):
            publisher = rospy.Publisher(topic["output"], PointCloud2, queue_size=1)
            rospy.Subscriber(topic["input"], PointCloud2,
                             lambda msg, pub=publisher: self.callback(msg, pub), queue_size=1)

    def odom_callback(self, msg):
        self.height = msg.pose.pose.position.z

    def callback(self, msg, publisher):
        minimum_z = self.min_z if self.min_z is not None else self.height
        if minimum_z is None:
            return
        points = [point for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True) if point[2] >= minimum_z]
        publisher.publish(point_cloud2.create_cloud_xyz32(msg.header, points))


if __name__ == "__main__":
    rospy.init_node("height_clip_pointcloud")
    HeightClipPointCloud()
    rospy.spin()

#!/usr/bin/env python3
"""Republish recorded obstacle points as occupancy visualization clouds."""

import rospy
from sensor_msgs.msg import PointCloud2


class PointCloudToOccupancy:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/fastLIO/non_ground_points")
        occupancy_topic = rospy.get_param("~occupancy_topic", "/b2/gvf/occupancy")
        inflate_topic = rospy.get_param("~inflate_topic", "/b2/gvf/occupancy_inflate")
        self.occupancy_pub = rospy.Publisher(occupancy_topic, PointCloud2, queue_size=1)
        self.inflate_pub = rospy.Publisher(inflate_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(input_topic, PointCloud2, self.callback, queue_size=1)

    def callback(self, msg):
        self.occupancy_pub.publish(msg)
        # The bag has no occupancy/ESDF obstacle surface to inflate offline.
        # Publish the measured obstacle cloud here so both RViz layers remain useful.
        self.inflate_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_occupancy")
    PointCloudToOccupancy()
    rospy.spin()

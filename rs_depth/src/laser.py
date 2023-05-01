#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs import point_cloud2
import math

def cloud_to_scan(cloud_msg):
    # Get the parameters from the cloud message
    points = list(point_cloud2.read_points(cloud_msg))
    width = cloud_msg.width
    height = cloud_msg.height
    angle_increment = 2*math.pi/width
    angle_min = -math.pi
    angle_max = math.pi
    range_min = 0.0
    range_max = 100.0
    scan_time = 1.0/30.0 # Example frequency of 30 Hz

    # Create the LaserScan message
    scan_msg = LaserScan()
    scan_msg.header = cloud_msg.header
    scan_msg.header.frame_id = "os1_sensor"
    scan_msg.angle_min = angle_min
    scan_msg.angle_max = angle_max
    scan_msg.angle_increment = angle_increment
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = scan_time
    scan_msg.range_min = range_min
    scan_msg.range_max = range_max

    # Populate the ranges field of the LaserScan message
    ranges = []
    for i in range(width):
        angle = angle_min + i*angle_increment
        range_val = range_max
        for point in points:
            x = point[0]
            y = point[1]
            z = point[2]
            r = math.sqrt(x*x + y*y + z*z)
            theta = math.atan2(y, x)
            if abs(theta - angle) < angle_increment/2.0 and r < range_val:
                range_val = r
        ranges.append(range_val)
    scan_msg.ranges = ranges

    return scan_msg

def cloud_callback(cloud_msg):
    scan_msg = cloud_to_scan(cloud_msg)
    scan_pub.publish(scan_msg)

rospy.init_node('cloud_to_scan')
scan_pub = rospy.Publisher('/os1_cloud_node/scan', LaserScan, queue_size=10)
cloud_sub = rospy.Subscriber('/os1_cloud_node/points', PointCloud2, cloud_callback)
rospy.spin()


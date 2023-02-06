#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print(f"received scan")
    exit(0)

rclpy.init(args=[])
node = rclpy.create_node("listener_scan")
sub = node.create_subscription(LaserScan, "/scan", callback, 10)
rclpy.spin_once(node)
print(f"scan timeout")
exit(1)


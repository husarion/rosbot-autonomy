#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import OccupancyGrid

def callback(msg):
    print(f"received map")
    exit(0)

rclpy.init(args=[])
node = rclpy.create_node("listener_map")
sub = node.create_subscription(OccupancyGrid, "/map", callback, 10)
rclpy.spin_once(node)
print(f"map timeout")
exit(1)


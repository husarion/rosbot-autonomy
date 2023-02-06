#!/usr/bin/env python3
import rclpy
from nav2_msgs.srv import SaveMap

rclpy.init(args=[])
node = rclpy.create_node("map_saver")
client = node.create_client(SaveMap, '/map_saver/save_map')

if not client.wait_for_service(timeout_sec=2.0):
    print(f"map timeout")
    exit (1)

request = SaveMap.Request()
request.free_thresh = 0.25
request.map_topic = "/map"
request.map_url = "/maps/map"
request.map_mode = "trinary"
request.image_format = "pgm"

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
print(f"map saved")
exit(0)
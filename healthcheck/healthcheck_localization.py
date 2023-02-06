#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

def callback(msg):
    print(f"received amcl_pose")
    exit(0)

rclpy.init(args=[])
latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

node = rclpy.create_node("amcl_pose_listener")
sub = node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", callback, qos_profile=latching_qos)
rclpy.spin_once(node)
print(f"amcl_pose timeout")
exit(1)

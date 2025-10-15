#!/usr/bin/env python3

# Copyright (c) 2025 - for information on the respective copyright owner
# see the NOTICE file

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')

        # Subscribe to PoseArray (absolute topic; can remap if needed)
        self.sub = self.create_subscription(
            PoseArray,
            'robot_pose',
            self.pose_callback,
            10
        )

        # Publish Odometry on a relative topic (respects namespace)
        self.pub = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster: odom -> base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def pose_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        pose = msg.poses[0]
        stamp = msg.header.stamp if (msg.header.stamp.sec or msg.header.stamp.nanosec) \
                else self.get_clock().now().to_msg()

        # Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'        # parent frame
        odom.child_frame_id = 'base_link'    # child frame
        odom.pose.pose = pose                # twist left zero
        self.pub.publish(odom)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

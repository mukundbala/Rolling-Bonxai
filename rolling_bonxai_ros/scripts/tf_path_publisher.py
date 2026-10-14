#!/usr/bin/env python3
"""Accumulate the trajectory of a TF frame into a nav_msgs/Path (for Foxglove / RViz).

Samples frame_id -> child_frame_id from TF every 1/sample_rate s, appends a pose when the frame has moved more
than min_distance, and republishes the whole path at publish_rate. The topic is transient_local, so a client
that connects late still gets the path. The path is cleared if the clock jumps backwards (bag restarted/looped).

Parameters: frame_id (map), child_frame_id (base_link), topic (/trajectory), sample_rate (10.0 Hz),
publish_rate (2.0 Hz), min_distance (0.05 m).
"""
import math

import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TfPathPublisher(Node):
    def __init__(self):
        super().__init__('tf_path_publisher')
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.child_frame_id = self.declare_parameter('child_frame_id', 'base_link').value
        topic = self.declare_parameter('topic', '/trajectory').value
        sample_rate = self.declare_parameter('sample_rate', 10.0).value
        publish_rate = self.declare_parameter('publish_rate', 2.0).value
        self.min_distance = self.declare_parameter('min_distance', 0.05).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Path, topic, qos)

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_stamp = None
        self.dirty = False
        self.create_timer(1.0 / sample_rate, self.sample)
        self.create_timer(1.0 / publish_rate, self.publish)

    def sample(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.child_frame_id, Time())
        except TransformException:
            return
        stamp = Time.from_msg(tf.header.stamp)
        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                self.get_logger().info('time jumped backwards; clearing path')
                self.path.poses.clear()
                self.tf_buffer.clear()
                self.dirty = True
            elif stamp == self.last_stamp:
                return
        self.last_stamp = stamp

        t = tf.transform.translation
        if self.path.poses:
            p = self.path.poses[-1].pose.position
            if math.dist((p.x, p.y, p.z), (t.x, t.y, t.z)) < self.min_distance:
                return

        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
        pose.pose.orientation = tf.transform.rotation
        self.path.poses.append(pose)
        self.dirty = True

    def publish(self):
        if not self.dirty:
            return
        self.path.header.stamp = self.path.poses[-1].header.stamp if self.path.poses else \
            self.get_clock().now().to_msg()
        self.pub.publish(self.path)
        self.dirty = False


def main():
    rclpy.init()
    node = TfPathPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

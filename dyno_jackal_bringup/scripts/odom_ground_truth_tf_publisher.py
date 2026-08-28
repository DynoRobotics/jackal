#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomGroundTruthTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_ground_truth_tf_publisher")

        self.declare_parameter("source_topic", "odom_ground_truth")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("use_odometry_header_frames", True)

        self.source_topic = self.get_parameter("source_topic").get_parameter_value().string_value
        self.parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        self.child_frame = self.get_parameter("child_frame").get_parameter_value().string_value
        self.use_odometry_header_frames = (
            self.get_parameter("use_odometry_header_frames").get_parameter_value().bool_value
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            self.source_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Publishing TF from '{self.source_topic}' as "
            f"'{self.parent_frame}' -> '{self.child_frame}'"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        parent = self.parent_frame
        child = self.child_frame

        if self.use_odometry_header_frames:
            if msg.header.frame_id:
                parent = msg.header.frame_id
            if msg.child_frame_id:
                child = msg.child_frame_id

        parent = parent.lstrip("/")
        child = child.lstrip("/")

        if not parent or not child:
            self.get_logger().warning("Skipping TF publish because frame id is empty")
            return

        tf_msg = TransformStamped()

        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            tf_msg.header.stamp = self.get_clock().now().to_msg()
        else:
            tf_msg.header.stamp = msg.header.stamp

        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomGroundTruthTfPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

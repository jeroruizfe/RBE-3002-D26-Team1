#!/usr/bin/env python3
from geometry_msgs.msg import PoseArray
from nav2_msgs.msg import ParticleCloud
from rclpy.node import Node

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class ParticleCloudRelay(Node):
    def __init__(self):
        super().__init__('particle_cloud_relay')

        self.get_logger().info('QoS relay started')

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT  # match AMCL
        )

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE     # match RViz
        )

        self.posearray_sub = self.create_subscription(
            PoseArray, '/particle_cloud', self.relay, sub_qos
        )

        # self.particlecloud_sub = self.create_subscription(
        #     ParticleCloud, '/particle_cloud', self.relay, sub_qos
        # )

        self.posearray_pub = self.create_publisher(
            PoseArray, '/particle_cloud_relay', pub_qos
        )

        # self.particlecloud_pub = self.create_publisher(
        #     ParticleCloud, '/particle_cloud_relay', pub_qos
        # )

    def relay(self, msg):
        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.poses = [p.pose for p in msg.particles]

        self.posearray_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)

    node = ParticleCloudRelay()
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
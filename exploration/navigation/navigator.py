#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.cb_group = ReentrantCallbackGroup()

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.check_confidence,
            10,
            callback_group=self.cb_group
        )

        self.global_loc_client = self.create_client(
            EmptySrv,
            '/reinitialize_global_localization',
            callback_group=self.cb_group
        )

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.curr_pose_pub = self.create_publisher(
            PoseStamped, # Type
            '/current_pose/localized',  # Topic
            10, # Buffer size 
        )

        self.should_spin = False
        self.converged = False

        self.startup_timer = self.create_timer(1.0, self.initialize)
        # Spin loop runs always at 10Hz, only publishes when should_spin is True
        self.pose_stamped = PoseStamped()
        self.spin_timer = None
        self.covariance = None

    def initialize(self):
        self.startup_timer.cancel()
        self.trigger_global_localization()

    def trigger_global_localization(self):
        if not self.global_loc_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Global localization service not available')
            return
        future = self.global_loc_client.call_async(EmptySrv.Request())
        future.add_done_callback(self.localization_done)

    def localization_done(self, future):
        try:
            future.result()
            self.get_logger().info('Particles scattered — starting spin to converge')
            self.should_spin = True
            self.spin_timer = self.create_timer(0.1, self.spin_tick)
        except Exception as e:
            self.get_logger().error(f'Global localization failed: {e}')

    def check_confidence(self, msg: PoseWithCovarianceStamped):
        self.pose_stamped.pose = msg.pose.pose
        self.pose_stamped.header = msg.header

        if self.converged:
            return
        cov = msg.pose.covariance  # flat array of 36 values
        x_var   = cov[0]           # [0,0]
        y_var   = cov[7]           # [1,1]
        yaw_var = cov[35]          # [5,5]

        mean_cov = (x_var + y_var + yaw_var) / 3.0
        self.get_logger().info(
            f'Covariance: x={x_var:.4f} y={y_var:.4f} yaw={yaw_var:.4f} mean={mean_cov:.4f}'
        )

        if mean_cov < 0.03:
            self.converged = True
            self.should_spin = False
            self.stop()
            if self.spin_timer:
                self.spin_timer.cancel()
            self.get_logger().info('AMCL converged — robot ready!')

            self.curr_pose_pub.publish(self.pose_stamped)




    def spin_tick(self):
        if not self.should_spin:
            return
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.angular.z = 0.63
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

import tf2_geometry_msgs

class DummyClient(Node):

    def __init__(self):
        super().__init__('Dummy_Path_Planner_Client')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.cb_group = ReentrantCallbackGroup() # allow for multiple things at once


        self.cli = self.create_client(GetPlan, 'request_plan',callback_group=self.cb_group)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


        self.req = GetPlan.Request()


        self.publisher_ = self.create_publisher(Path, '/nav_path', 10)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.send_request,
            10,
            callback_group=self.cb_group)

        self.px = 0.0
        self.py = 0.0

        self.odom_sub = self.create_subscription(
            PoseStamped, # Type
            '/current_pose',  # Topic
            self.update_pose, # Callback function (function that is called)
            10, # Buffer size 
            callback_group = self.cb_group
        )

        self.current_goal = None
        self.current_path = None  # store the last published path

        self._blocked_count = 0
        self._BLOCKED_THRESHOLD = 3
        self._last_replan_x = 0.0
        self._last_replan_y = 0.0
        self._MIN_TRAVEL_BEFORE_REPLAN_M = 0.3

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.on_map_update,
            10,
            callback_group=self.cb_group
        )


    def update_pose(self, msg):
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y
    
    def send_request(self, goal):
        self.current_goal = goal

        start_pose = PoseStamped()
        start_pose.header.frame_id = 'odom'
        start_pose.header.stamp = rclpy.time.Time().to_msg()  # use latest transform
        start_pose.pose.position.x = self.px
        start_pose.pose.position.y = self.py
        start_pose.pose.orientation.w = 1.0

        try:
            start_pose = self._tf_buffer.transform(start_pose, 'map', rclpy.duration.Duration(seconds=0))
        except Exception as e:
            self.get_logger().warn(f'Could not transform start pose to map frame: {e}')

        self.req.start = start_pose
        self.req.goal = goal

        self.get_logger().info("starting service")

        future = self.cli.call_async(self.req)

        future.add_done_callback(self.publish)  # ← non-blocking
        self.get_logger().info("ending callback")

        self._blocked_count = 0
        self._last_replan_x = self.px
        self._last_replan_y = self.py
        
    def publish(self, future):
        plan = future.result().plan
        if not plan.poses:
            self.get_logger().warn('Path planner returned empty path')
            return

        pruned_poses = []
        for pose in plan.poses:
            dx = pose.pose.position.x - self.px
            dy = pose.pose.position.y - self.py
            if (dx**2 + dy**2) ** 0.5 >= 0.15:
                pruned_poses.append(pose)

        if not pruned_poses:
            self.get_logger().info('All waypoints already passed, skipping')
            return

        robot_pose = PoseStamped()  # robot's real map position
        robot_pose.header.frame_id = 'map'
        robot_pose.header.stamp = self.get_clock().now().to_msg()
        robot_pose.pose.position.x = self.px
        robot_pose.pose.position.y = self.py
        robot_pose.pose.orientation.w = 1.0

        # robot_pose = self._tf_buffer.transform(robot_pose, 'map', rclpy.duration.Duration(seconds=1))
        

        plan.poses = [robot_pose] + pruned_poses
        self.current_path = plan
        self.publisher_.publish(plan)

    def on_map_update(self, msg):
        if self.current_goal is None or self.current_path is None:
            return

        # Store map for collision checking
        map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        # Check if any waypoint in the current path is now blocked
        

        self.get_logger().info('Path persistently blocked — replanning')
        self._blocked_count = 0
        self.current_path = None
        self.send_request(self.current_goal)



def main(args=None):
    rclpy.init(args=args)

    node = DummyClient()
    executor = MultiThreadedExecutor()

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
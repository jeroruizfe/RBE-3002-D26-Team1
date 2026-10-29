#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path, GridCells
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from std_msgs.msg import Empty as EmptyMsg


import tf2_geometry_msgs
from math import sqrt, pow

class MapClient(Node):

    def __init__(self):
        super().__init__('Map_Client')

        self.cb_group = ReentrantCallbackGroup()

        self.cli = self.create_client(GetPlan, 'request_frontier', callback_group=self.cb_group)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.pub_failed_frontier = self.create_publisher(PoseStamped, '/frontier_failed', 10)

        self.req = GetPlan.Request()
        self.req.start.header.frame_id = "map"
        self.req.goal.header.frame_id = "map"
        self.req.tolerance = 0.0

        self.last_goal_distance = float('inf')
        self.no_progress_count = 0

        self.pub_pose_stamped = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.pub_replan = self.create_publisher(PoseStamped, '/replan', 10)
        self.pub_mapping_complete = self.create_publisher(EmptyMsg, '/mapping_complete', 10)

        # FIX 1: use a pending flag to prevent the map callback from firing a
        # second service call while a future is already in flight.  The old
        # code only checked self.navigating, which is set INSIDE publish()
        # (the async callback) — so two map updates arriving before the first
        # future completes would both pass the guard and call the service twice,
        # causing two competing navigation goals.
        self.navigating = False
        self._service_pending = False

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map/safe',
            self.send_request,
            10,
            callback_group=self.cb_group)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_reached',
            self.handle_goal_reached,
            10,
            callback_group=self.cb_group
        )

        self.curr_goal = None
        self.current_pose = PoseStamped()
       
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.update_pose,
            10,
            callback_group=self.cb_group
        )

        self.visited_frontiers = []

        self.visited_sub = self.create_subscription(
            GridCells,
            '/map/frontiers',
            self.update_vistied,
            10,
            callback_group=self.cb_group
        )

        self.initial_pose = None
        self.exploration_done = False


    def update_pose(self, msg: PoseStamped):
        if self.initial_pose is None:
            self.initial_pose = msg
        self.current_pose = msg

    def update_vistied(self, msg: GridCells):
        self.visited_frontiers = []
        for point in msg.cells:
            self.visited_frontiers.append((point.x, point.y))

    def send_request(self, frontier_map):
        
        if self.navigating and self.curr_goal is not None:
            goal_x = self.curr_goal.pose.position.x
            goal_y = self.curr_goal.pose.position.y
            robot_x = self.current_pose.pose.position.x
            robot_y = self.current_pose.pose.position.y

            dist = sqrt(pow((goal_x - robot_x), 2) + pow((goal_y - robot_y), 2))

            # Checking if goal frontier is already an identified visited frontier
            # Threshold in case coordinates are not exactly the same
            visited_goal = any(
                abs(goal_x - visited_x) < 0.1 and abs(goal_y - visited_y) < 0.1
                for visited_x, visited_y in self.visited_frontiers
            )

            # if dist > 0.3:
            #     heading_to_goal = atan2(goal_y - robot_y, goal_x - robot_x)
            #     angle_diff = heading_to_goal - self.robot_yaw
            #     # normalise...
            #     if abs(angle_diff) < 1.2:
            #         return  

            if visited_goal and dist < 0.35:
                self.get_logger().info("Goal is already visited, replanning")
                self.navigating = False
                self.curr_goal = None
                self.no_progress_count = 0
                self.last_goal_distance = float('inf')
        
        
        # FIX 1 (continued): gate on both flags
        if self.navigating or self._service_pending:
            self.get_logger().info("Already navigating, ignoring map update")
            return

        self.req.start = self.current_pose
        self._service_pending = True
        self.get_logger().info("Requesting frontier from map_explore service")
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.publish)


    def publish(self, future):
        self._service_pending = False  # FIX 1: clear pending flag first

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        if not result.plan.poses:
            if self.exploration_done:
                self.get_logger().info('No frontier returned, already returning home')
                return
            self.get_logger().info('No frontiers left, returning to start')
            self.exploration_done = True
            self.pub_mapping_complete.publish(EmptyMsg())
            if self.initial_pose is not None:
                self.pub_pose_stamped.publish(self.initial_pose)
                self.curr_goal = self.initial_pose
                self.last_goal_distance = float('inf')
                self.no_progress_count = 0
                self.navigating = True
            return

        goal = result.plan.poses[0]
        self.get_logger().info(
            f"Navigating to frontier at "
            f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
        )
        self.pub_pose_stamped.publish(goal)
        self.curr_goal = goal
        self.last_goal_distance = float('inf')
        self.no_progress_count = 0
        self.navigating = True


    def handle_goal_reached(self, msg):
        self.get_logger().info("Goal reached — ready for next frontier")
        self.navigating = False
        self.curr_goal = None


def main(args=None):
    rclpy.init(args=args)

    node = MapClient()
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
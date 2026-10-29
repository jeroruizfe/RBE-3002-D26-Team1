#!/usr/bin/env python3
from __future__ import annotations
from typing import List, Tuple
import numpy.typing as npt

import os
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node, ReentrantCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import DurabilityPolicy, HistoryPolicy, QoSProfile, TransformListener
from scipy.spatial.transform import Rotation
from nav2_msgs.srv import SaveMap

import numpy as np
from math import sqrt, pow, sin, cos, atan2, pi
from nav_msgs.srv import GetPlan
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData, GridCells, Odometry
import threading
# from tf_transformations import euler_from_quaternion

class MapExplore(Node):
    def __init__(self):
        super().__init__('map_explore')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cb_group = ReentrantCallbackGroup()
        
        self.qos_policy = QoSProfile(
            history    = HistoryPolicy.KEEP_LAST,
            depth      = 10,
            durability = DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.safe_map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map/safe',
            self.update_map,
            self.qos_policy,
            callback_group = self.cb_group
        )

        self.failed_frontier_sub = self.create_subscription(
            PoseStamped,
            '/frontier_failed',
            self.handle_failed_frontier,
            10,
            callback_group=self.cb_group
        )

        self.risk_map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map/risk',
            self.update_risk_map,
            self.qos_policy,
            callback_group = self.cb_group
        )

        self.odom_subscriber = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.update_pose,
            10,
            callback_group=self.cb_group
        )

        self.frontier_pub = self.create_publisher(
            GridCells, 
            '/map/frontiers', 
            10
        )

        self.process_GetFrontier_service = self.create_service(
            GetPlan,
            'request_frontier',
            self.get_frontier,
            callback_group = self.cb_group
        )

        self.client = self.create_client(SaveMap, '/map_saver/save_map')

        self.map = np.zeros((0,0))
        self.risk_map = np.zeros((0,0))
        self.map_info = MapMetaData()
        
        self._map_lock = threading.Lock()
        
        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0

        self.last_map_free_cells = 0

        # visited_frontiers stores world-space (x, y) tuples so the blacklist
        # survives map resizes (cell indices shift as the map grows).
        self.visited_frontiers: set = set()
        self.VISITED_RADIUS_M = 0.3  # metres — tune if needed

        self.robot_yaw = 0.0

    # ------------------------------------------------------------------
    # Map / pose updates
    # ------------------------------------------------------------------

    def update_map(self, msg: OccupancyGrid):
        new_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        with self._map_lock:
            self.map = new_map
            self.map_info = msg.info

        self.send_request('RBE-3002-D26-Team1/src/exploration/maps/explored_map_test')
        self.get_logger().info("Map updated")
        free_cells = int(np.sum(new_map == 0))
        if free_cells > self.last_map_free_cells + 200:
            self.visited_frontiers.clear()
            self.last_map_free_cells = free_cells

    def update_risk_map(self, msg: OccupancyGrid):
        
        new_risk_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        with self._map_lock:
            self.risk_map = new_risk_map
        self.get_logger().info("Risk map updated")

    def update_pose(self, msg: PoseStamped):
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y

    def send_request(self, map_name):
        maps_dir = os.path.join(
            get_package_share_directory('exploration'),
            'maps'
        )

        map_path = os.path.join(maps_dir, 'explored_map')

        request = SaveMap.Request()
        request.map_topic = '/map'
        request.map_url = map_path
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        
        # Call service asynchronously
        return self.client.call_async(request)

    def handle_pose(self, msg):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------

    def find_frontiers(self, map_snap: OccupancyGrid, map_info_snap) -> List[Tuple[int, int]]:
        frontier = []
        for i in range(map_info_snap.width):
            for j in range(map_info_snap.height):
                if map_snap[j, i] == 0:
                    for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
                        new_x, new_y = i + dx, j + dy
                        if 0 <= new_x < map_info_snap.width and 0 <= new_y < map_info_snap.height:
                            if map_snap[new_y, new_x] == -1:
                                if (i, j) not in frontier:
                                    frontier.append((i, j))
        return frontier

    # ------------------------------------------------------------------
    # Visited-frontier helpers (world space)
    # ------------------------------------------------------------------

    def _cell_to_world(self, col: int, row: int, map_info_snap) -> Tuple[float, float]:
        res = map_info_snap.resolution
        wx = (col + 0.5) * res + map_info_snap.origin.position.x
        wy = (row + 0.5) * res + map_info_snap.origin.position.y
        return wx, wy

    def _is_visited(self, col: int, row: int, map_info_snap) -> bool:
        wx, wy = self._cell_to_world(col, row, map_info_snap)
        for vx, vy in self.visited_frontiers:
            if (wx - vx) ** 2 + (wy - vy) ** 2 < self.VISITED_RADIUS_M ** 2:
                return True
        return False

    def _mark_visited(self, col: int, row: int, map_info_snap):
        wx, wy = self._cell_to_world(col, row, map_info_snap)
        self.visited_frontiers.add((wx, wy))

    # ------------------------------------------------------------------
    # Frontier evaluation
    # ------------------------------------------------------------------

    def evaluate_frontiers(self, frontiers: List[Tuple[int, int]], map_snap, map_info_snap, risk_snap, px, py, robot_yaw):
        """Return the (col, row) of the best unvisited frontier, or None."""
        min_cost = float('inf')
        min_point = None
        self.get_logger().info("Evaluating Frontiers")

        res = map_info_snap.resolution
        origin_x = map_info_snap.origin.position.x
        origin_y = map_info_snap.origin.position.y

        map_h, map_w = map_snap.shape

        robot_gx = int((px - origin_x) / res)
        robot_gy = int((py - origin_y) / res)
        robot_gx = max(0, min(robot_gx, map_w - 1))
        robot_gy = max(0, min(robot_gy, map_h - 1))

        num_frontiers = len(frontiers)
        low_frontier_mode = num_frontiers < 6

        for frontier in frontiers:
            col, row = frontier[0], frontier[1]

            # FIX 1: skip visited frontiers using world-space radius check
            if self._is_visited(col, row, map_info_snap):
                continue

            # Skip map-boundary cells
            if row < 1 or row >= map_h - 1 or col < 1 or col >= map_w - 1:
                continue

            # Skip if occupied in the safe map
            if map_snap[row, col] != 0:
                continue

            area_around = np.array(map_snap[row-1:row+2, col-1:col+2])
            unknown_neighbours = int(np.count_nonzero(area_around == -1))

            # Euclidean distance in grid cells
            dist_cost = sqrt((col - robot_gx)**2 + (row - robot_gy)**2)
            dist_cost = min(dist_cost / 100, 1.0)

            if low_frontier_mode:
                cost = dist_cost

            else:
                # FIX 2: require at least 2 unknown neighbours to filter out tiny
                # 1-cell frontier slivers that immediately re-appear after being visited
                if unknown_neighbours < 2:
                    continue

                info_cost = 1.0 - (unknown_neighbours / 9.0)

                # Risk cost
                if risk_snap.size > 0:
                    risk_cost = (float(risk_snap[robot_gy, robot_gx]) +
                                float(risk_snap[row, col])) / 2.0
                    risk_cost = min (risk_cost / 100.0, 1.0)
                else:
                    risk_cost = 0.0

                # Forward frontier cost
                heading_angle = atan2((row - robot_gy), (col - robot_gx))
                heading_diff = heading_angle - robot_yaw

                if (heading_diff > pi):
                    heading_diff -= 2*pi
                elif (heading_diff < -pi):
                    heading_diff += 2*pi

                forward_cost = abs(heading_diff) / pi

                # FIX 3: weight distance so the robot commits to one frontier
                # rather than oscillating between equidistant ones
                cost = 0.05 * info_cost + 0.3 * dist_cost + 1 * risk_cost + 0.4 * forward_cost
                
            if cost < min_cost:
                min_point = frontier
                min_cost = cost

        return min_point

    # ------------------------------------------------------------------
    # Step toward frontier
    # ------------------------------------------------------------------

    def step_toward_frontier(self, min_point: Tuple[int, int], map_info_snap, px, py) -> Tuple[int, int]:
        percentage = 1.0  # go all the way to the frontier

        res = map_info_snap.resolution
        origin_x = map_info_snap.origin.position.x
        origin_y = map_info_snap.origin.position.y

        grid_x = int((px - origin_x) / res)
        grid_y = int((py - origin_y) / res)

        dx = min_point[0] - grid_x
        dy = min_point[1] - grid_y

        new_x = int(grid_x + dx * percentage)
        new_y = int(grid_y + dy * percentage)

        if abs(dx) < 3 and abs(dy) < 3:
            new_x, new_y = min_point

        return (new_x, new_y)

    # ------------------------------------------------------------------
    # Failed-frontier handler
    # ------------------------------------------------------------------

    def handle_failed_frontier(self, msg: PoseStamped):
        """Blacklist the failed frontier in world space."""
        wx = msg.pose.position.x
        wy = msg.pose.position.y
        # FIX 4: store world-space coords so blacklist survives map resizes
        self.visited_frontiers.add((wx, wy))
        self.get_logger().info(f'Blacklisted failed frontier at world ({wx:.2f},{wy:.2f})')

    # ------------------------------------------------------------------
    # Build response pose
    # ------------------------------------------------------------------

    def build_pose_stamped(self, frontier: Tuple[int, int], map_info_snap) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = (frontier[0] + 0.5) * map_info_snap.resolution + map_info_snap.origin.position.x
        pose_stamped.pose.position.y = (frontier[1] + 0.5) * map_info_snap.resolution + map_info_snap.origin.position.y
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def get_frontier(self, request, response):
        with self._map_lock:
            if self.map.size == 0:
                self.get_logger().info("No map available")
                return response

            # Data values at a snapshot
            map_snap = self.map.copy()
            map_info_snap = self.map_info
            risk_snap = self.risk_map.copy() if self.risk_map.size > 0 else np.zeros((0,0))
            px, py = self.px, self.py
            robot_yaw = self.robot_yaw

        frontiers = self.find_frontiers(map_snap, map_info_snap)
        if not frontiers:
            self.get_logger().info("No frontiers found")
            return response

        # Publish all frontiers for visualisation
        res = map_info_snap.resolution
        origin_x = map_info_snap.origin.position.x
        origin_y = map_info_snap.origin.position.y
        gc = GridCells()
        gc.header.frame_id = 'map'
        gc.header.stamp = self.get_clock().now().to_msg()
        gc.cell_width = res
        gc.cell_height = res
        for (fx, fy) in frontiers:
            p = Point()
            p.x = float((fx + 0.5) * res + origin_x)
            p.y = float((fy + 0.5) * res + origin_y)
            p.z = 0.0
            gc.cells.append(p)
        self.frontier_pub.publish(gc)

        best_frontier = self.evaluate_frontiers(frontiers, map_snap, map_info_snap, risk_snap, px, py, robot_yaw)

        # FIX 5: if all frontiers are exhausted/visited, clear and retry once
        if best_frontier is None:
            self.get_logger().info("All frontiers visited — clearing blacklist and retrying")
            self.visited_frontiers.clear()
            best_frontier = self.evaluate_frontiers(frontiers, map_snap, map_info_snap, risk_snap, px, py, robot_yaw)
            if best_frontier is None:
                self.get_logger().info("Still no frontier after clearing — exploration may be complete")
                return response

        # Mark as visited BEFORE returning so a rapid second service call
        # cannot pick the same frontier again
        self._mark_visited(best_frontier[0], best_frontier[1], map_info_snap)

        #frontier = self.step_toward_frontier(best_frontier, map_info_snap, px, py)

        response.plan.poses = [self.build_pose_stamped(best_frontier, map_info_snap)]
        self.get_logger().info(
            f"Targeting frontier cell {best_frontier} -> "
            f"world ({response.plan.poses[0].pose.position.x:.2f}, "
            f"{response.plan.poses[0].pose.position.y:.2f})"
        )
        return response


def main(args=None):
    print("Running map explore node")
    rclpy.init(args=args)
    node = MapExplore()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        print("Navigation node is spinning")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down map explore node")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
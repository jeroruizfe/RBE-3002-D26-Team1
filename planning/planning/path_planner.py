#!/usr/bin/env python3
from __future__ import annotations
from typing import List, Tuple
import numpy.typing as npt

from rclpy.node import Node, ReentrantCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import DurabilityPolicy, HistoryPolicy, QoSProfile, TransformListener

import numpy as np
import math
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose
from std_srvs.srv import SetBool  # or a custom srv, but simplest: use GetPlan reusing start/goal
from nav_msgs.srv import GetPlan
from  nav_msgs.msg import OccupancyGrid, Path, MapMetaData, GridCells, Odometry
from nav_msgs.srv import GetPlan
from scipy.spatial.transform import Rotation
import cv2 as cv
from scipy.ndimage import binary_dilation

from priority_queue import PriorityQueue 

import tf2_geometry_msgs

class GraphNode():
    def __init__(self,pose,parent,cost):
        self.pose = pose
        self.parent = parent
        self.cost = cost
    
    def __eq__(self,other):
        return self.pose[0] == other.pose[0] and \
               self.pose[1] == other.pose[1] and \
               self.pose[2] == other.pose[2]
    
    def __lt__(self,other):

        return id(self) < id(other)


class PathPlanner(Node):
    def __init__(self):
        # INDIVIDUAL 

        # Initialize the node and call it "path_planner"
        super().__init__('path_planner')

        self.cb_group = ReentrantCallbackGroup()

        # Create Quality of Service (QoS) policy. Include a profile, depth, and durablilty policy. 
        self.qos_policy = QoSProfile(
            history = HistoryPolicy.KEEP_LAST,
            depth = 1000,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribe to the map topic
        self.sub_map = self.create_subscription( 
            OccupancyGrid, # Type
            '/map',  # Topic
            self.handle_map, # Callback function (function that is called)
            self.qos_policy, # QoS policy
            callback_group = self.cb_group
        )            

        # Subscribe to the clicked_point topic
        self.sub_clicked_point = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.handle_clicked_point,
            10,
            callback_group = self.cb_group
        )

        # Create a new publisher for '/map/safe' that publishes a message of type OccupancyGrid
        self.pub_safe_map = self.create_publisher( 
            OccupancyGrid,
            '/map/safe',
            self.qos_policy
        )

        # Create a new publisher for the topic '/map/risk' that publishes a message of type OccupancyGrid
        self.pub_risk_map = self.create_publisher(
            OccupancyGrid,
            'map/risk',
            self.qos_policy
        )

        self.sub_curr_pose = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.update_pose,
            1000,
            callback_group = self.cb_group
        )

        # Service to check if there is a direct line of sight to optimize the path
        self.validate_traj_service = self.create_service(
            GetPlan,
            'validate_trajectory',
            self.validate_trajectory_service_handler,
            callback_group=self.cb_group
        )

        # TODO: Create other publishers with varying message types (GridCells, etc) for visualizing data in Rviz
        self.visited_publisher = self.create_publisher(GridCells, '/path_planner/visited', 10)
        self.path_publisher = self.create_publisher(Path, '/nav_path', 10)


        # Create service to process GetPlan service requests. See https://docs.ros.org/en/humble/p/nav_msgs/srv/GetPlan.html
        self.process_GetPlan_service = self.create_service(
            GetPlan,
            'request_plan',
            self.plan_path,
            callback_group = self.cb_group
        )

        self.map_service = self.create_service(
            GetPlan,
            'request_map',
            self.handle_map,
            callback_group = self.cb_group
        )


        self.get_logger().info("Path planner node initalized!")

        # numpy arrays of cell values
        self.map = np.zeros((0,0))
        self.safe_map = np.zeros((0,0))
        self.risk_map = np.zeros((0,0))
        self.map_info = None

        # Setup TF Buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0


    def validate_trajectory_service_handler(self, request, response):
        """
        Reuses GetPlan: start = start_pose, goal = goal_pose.
        Returns an empty path if invalid, non-empty if valid.
        """
        is_valid = self.validate_trajectory(
            request.start.pose,
            request.goal.pose,
            lin_vel=0.2,
            ang_vel=0.0
        )
        if is_valid:
            response.plan.poses = [request.start, request.goal]  # non-empty = valid
        # empty plan = invalid (default)
        return response

    @staticmethod
    def obstacle_expansion(original_map:npt.NDArray[np.int32], padding:int):
        """
        Expands obstacles to be "padding" larger in all directions
        :param original_map     ndarray     Map of obstacles
        :param padding          int         Number of cells from obstacle to consider unsafe
        :return                 ndarray     Map of safe configuration space.
        """
        # INDIVIDUAL
        kernel = np.ones(((1 + (padding * 2)), (1 + (padding * 2))))

        obstacles = (original_map == 100) # Takes only the occupied cells in the map
        
        dilatied_map = binary_dilation(obstacles, kernel).astype(int) # Expands the occupied cells
        
        safe_map = original_map.copy()    # Copies the original map
        safe_map[dilatied_map >= 1] = 100 # Sets the expanded cells to be occupied in the safe map

        return safe_map

    def update_pose(self, msg):
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y

    def compute_risk(self, map: npt.NDArray[np.int32]) -> npt.NDArray[np.float64]:
        risk = np.full(map.shape, np.inf)

        queue = []

        # Queue obstacle cells (distance = 0)
        for x in range(map.shape[0]):
            for y in range(map.shape[1]):
                if map[x, y] == 100:
                    risk[x, y] = 0
                    queue.append((x, y))

        # BFS outward from obstacles
        while queue:
            cx, cy = queue.pop(0)
            # Get neighbors of 8
            for dx, dy in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
                nx, ny = cx + dx, cy + dy  # cx=row, cy=col
                if 0 <= nx < self.map.shape[0] and 0 <= ny < self.map.shape[1]:
                    point = (nx, ny)
                    if risk[point] > risk[cx, cy] + 1:
                        risk[point] = risk[cx, cy] + 1
                        queue.append(point)

        return risk

    def remove_isolated_obstacle(self, map: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Removes obstacle cells that have fewer than min_neighbors obstacle neighbors.
        Eliminates single-cell LIDAR noise before risk computation.
        :param map              The occupancy grid (100 = obstacle), which is the risk map.
            1 = the cell itself is an identified obstacle
            2 = cells has only 1 obstacle neighbor 
        """
        filtered_map = map.copy()
        free_cell = (map == 0)
        obstacle_cells = (map == 100)

        for y in range(map.shape[0]):
            for x in range(map.shape[1]):
                if obstacle_cells[y, x]:
                    obstacles = 1
                    if (0 <= y and y < self.map.shape[0] and 
                        0 <= x and x < self.map.shape[1]):
                        for dx, dy in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
                            nx, ny = x + dx, y + dy
                            if (0 <= ny < self.map.shape[0] and 
                                0 <= nx < self.map.shape[1]):
                                if obstacle_cells[ny, nx]:
                                    obstacles += 1

                        if obstacles < 2: # If there are only 2 obstacles neighboring each other
                            filtered_map[y, x] = 0 # Make it free

        return filtered_map

    def handle_map(self, map:OccupancyGrid, response):
        """
        Recieve raw map, convert to numpy array, expand obstacles.
        Save safe map and republish. 
        :param map      OccupancyGrid   The current map.
        """
        # INDIVIDUAL
        self.get_logger().info("map received")

        raw_map = np.asarray(map.data).reshape(map.info.height, map.info.width)
        filtered_map = self.remove_isolated_obstacle(raw_map)
        safe_map = self.obstacle_expansion(filtered_map, 6)
        risk_map = self.compute_risk(safe_map)

        planning_map = safe_map.copy()
        planning_map[planning_map == -1] = 100

        # Normalize risk into a publishable OccupancyGrid 
        finite_vals = risk_map[np.isfinite(risk_map)]
        max_risk = finite_vals.max() if finite_vals.size > 0 else 0

        if max_risk == 0:
            normalized = np.zeros(risk_map.shape, dtype=int)
        else:
            normalized = ((1 - risk_map / max_risk) * 100).astype(int)
            normalized[filtered_map == 100] = 100  # obstacles stay at 100
            normalized = np.clip(normalized, 0, 100)

        # Build OccupancyGrid messages 
        safe_grid = OccupancyGrid()
        safe_grid.info = map.info
        safe_grid.header = map.header
        safe_grid.data = safe_map.flatten().tolist()

        risk_grid = OccupancyGrid()
        risk_grid.info = map.info
        risk_grid.header = map.header
        risk_grid.data = normalized.flatten().tolist()

        # Atomic commit — write to self.* only here 
        self.map = filtered_map
        self.safe_map = planning_map
        self.risk_map = risk_map
        self.map_info = map.info

        # Publish 
        self.pub_safe_map.publish(safe_grid)
        self.pub_risk_map.publish(risk_grid)

        if response is not None:
            response = risk_grid
            return response

    @staticmethod
    def grid_to_world(mapdata: MapMetaData, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [MapMetaData] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        # INDIVIDUAL
        resolution = mapdata.resolution # The size of each cell in the occupancy grid (in meters per cell)
        origin_x = mapdata.origin.position.x # The x-coordinate of the origin of the occupancy grid in the world frame
        origin_y = mapdata.origin.position.y # The y-coordinate of the origin of the occupancy grid in the world frame

        world_x = origin_x + (p[0] + 0.5) * resolution # The x-coordinate of the center of the cell in the world frame
        world_y = origin_y + (p[1] + 0.5) * resolution # The y-coordinate of the center of the cell in the world frame

        return Point(x=world_x, y=world_y, z=0.0)

        
    @staticmethod
    def world_to_grid(mapdata: MapMetaData, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [MapMetaData] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        # INDIVIDUAL
        resolution = mapdata.resolution # The size of each cell in the occupancy grid (in meters per cell)
        origin_x = mapdata.origin.position.x # The x-coordinate of the origin of the occupancy grid in the world frame
        origin_y = mapdata.origin.position.y # The y-coordinate of the origin of the occupancy grid in the world frame

        grid_x = int((wp.x - origin_x) / resolution) # The x-coordinate of the cell in the occupancy grid
        grid_y = int((wp.y - origin_y) / resolution) # The y-coordinate of the cell in the occupancy grid

        return (grid_y, grid_x)


    @staticmethod
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        # INDIVIDUAL
        dist = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) # Pythagorean theorem 
        
        return dist
        

    def build_path_message(self, path: list[tuple[int, int]]) -> Path:
        """
        Converts a list of cell coordinates to a Path Message
        :param path     [(int,int)]     The cell coordinates corresponding to the current map
        :return         Path            
        """
        # INDIVIDUAL
        new_path = Path() # Create a new Path message
        new_path.header.frame_id = 'map'
        new_path.header.stamp = self.get_clock().now().to_msg()


        for cell in path: # For each cell in the path
            point = self.grid_to_world(self.map_info, cell) # Convert the cell coordinate to a world coordinate

            pose = PoseStamped() # Create a new PoseStamped message for the current cell
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = point

            new_path.poses.append(pose) # Add the pose to the Path message

        return new_path
        

    def neighbors_of_4(self,p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the safe 4-neighbors cells of (x,y) in the occupancy grid.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable in 4 cardinal directions.
        """
        # INDIVIDUAL
        neighbors = [] # List to store the valid neighbors
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]: # Loop through the 4 cardinal directions
            new_x, new_y = p[0] + dx, p[1] + dy      # Calculate the new coordinates
            
            if 0 <= new_x < self.map.shape[0] and 0 <= new_y < self.map.shape[1]: # Check if coordinates are in bounds
                if self.map[new_x, new_y] != 100:      # Check if cell is walkable
                    neighbors.append((new_x, new_y, 0)) # Add valid neighbor to the list
        
        return neighbors


    def neighbors_of_8(self, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the safe 8-neighbors cells of (x,y) in the occupancy grid.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable in 4 cardinal directions.
        """
        # INDIVIDUAL
        neighbors = [] # List to store the valid neighbors
        for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]: # Loop through the 8 directions
            new_x, new_y = p[0] + dx, p[1] + dy      # Calculate the new coordinates
            
            if 0 <= new_x < self.map.shape[1] and 0 <= new_y < self.map.shape[0]:
                if self.safe_map[new_y, new_x] != 100:
                    neighbors.append((new_x, new_y, 0)) # Add valid neighbor to the list
        
        return neighbors


    def get_edge_cost(self,p1: tuple[int, int, int],p2: tuple[int, int, int], risk_map=False):
        """
        Compute cost to traverse between 2 nodes.
        :param p1       [(int, int, int)]    The coordinate in the grid of start.
        :param p2       [(int, int, int)]    The coordinate in the grid of end.
        :param risk_map     boolean          The flag representing if average risk should be included in the cost    

        """
        # INDIVIDUAL
        cost = self.euclidean_distance(p1[:2], p2[:2])
        if risk_map:
            r1 = float(self.risk_map[p1[1], p1[0]])
            r2 = float(self.risk_map[p2[1], p2[0]])
            avg_dist = (r1 + r2) / 2.0

            RISK_WEIGHT = 16.5
            DECAY = 0.15  # higher = steeper drop-off near obstacles
            cost += RISK_WEIGHT * math.exp(-DECAY * avg_dist)
        return cost


    def handle_clicked_point(self, msg: PointStamped):

        clicked_point = self.world_to_grid(self.map_info, msg.point) # Convert the clicked point from world to grid coordinates

        if self.map[int(clicked_point[0]), int(clicked_point[1])] == 0: # Check if the clicked point is in a walkable cell
            self.get_logger().info(f"Grid cell {clicked_point}. Valid point")
        else:
            self.get_logger().info(f"Grid cell {clicked_point}. Invalid point")

        


    def draw_visited(self,visited):
        """world_to_grid
        Draw set of visited node as GridCells message.
        """
        cells = GridCells()
        cells.cell_width = self.map_info.resolution
        cells.cell_height = self.map_info.resolution
        cells.header.stamp = self.get_clock().now().to_msg()
        cells.header.frame_id = 'map'
        for index in visited:
            p = self.grid_to_world(self.map_info,index[:2])            
            cells.cells.append(p)
        self.visited_publisher.publish(cells)


    def build_path(self,final_node: GraphNode):
        """
        Given goal node from search tree, build path from start to goal to get there
        """

        current_node = final_node
        path = []
        while current_node is not None:
            path.append(current_node.pose)
            current_node= current_node.parent
        return path[::-1]


    def a_star(self, start: tuple[int, int, int], goal: tuple[int, int, int]) -> list[tuple[int, int, int]]:
        """
        Runs A* search from start coordinate to end coordinate using class member map.
        :param start    [[(int,int,int)]]     start coordinate in map (x, y, theta (degrees))
        :param goal     [[(int,int,int)]]      goal coordinate in map (x, y, theta (degrees))
        :return 

        """
        # GROUP
        if self.safe_map[int(goal[1]), int(goal[0])] == 100: # Check if the goal point is NOT in a walkable cell
            self.get_logger().info(f"Grid cell {goal}. Invalid point")
            return
        
        Frontier = PriorityQueue() # Initialize the frontier as a priority queue
        Frontier.put(GraphNode(start, None, 0), 0) # Add the start node to the frontier with a priority of 0
        visited = set() # Initialize the visited set to keep track of visited nodes

        while not Frontier.empty(): # Loop until the frontier is empty
            current_node = Frontier.get() # Get the node in the frontier with the lowest f(n) = g(n) + h(n)
            visited.add(current_node.pose) # Add the current node to the visited set

            if (current_node.pose[0] == goal[0] and current_node.pose[1] == goal[1]): # Check if we have reached the goal
                return self.build_path(current_node) # If we have reached the goal, build and return the path from start to goal
            
            for neighbor in self.neighbors_of_8([current_node.pose[0], current_node.pose[1]]): # Loop through the neighbors of the current node
                if neighbor in visited: # If the neighbor has already been visited, skip it
                    continue
                
                g_cost = current_node.cost + self.get_edge_cost(current_node.pose, neighbor, True) # Calculate g(n) for the neighbor
                h_cost = self.euclidean_distance(neighbor[:2], goal[:2]) # Calculate h(n) for the neighbor
                f_cost = g_cost + h_cost # Calculate f(n) for the neighbor

                Frontier.put(GraphNode(neighbor, current_node, g_cost), f_cost) # Add the neighbor to the frontier with its f(n) as the priority
            
            # self.draw_visited(visited) # Visualize visited nodes in Rviz
        

    def plan_path(self, request, response): 
        """
        Plans a path between the current pose and the goal message locations.
        Internally uses A* to plan the optimal path.
        :param request  nav_msgs.srv._get_plan.GetPlan_Request  Start and End Pose for plan
        :param response nav_msgs.srv._get_plan.GetPlan_Response 
        :return         nav_msgs.srv._get_plan.GetPlan_Response     
        """
        # GROUP
        # Add error handling for if there is no map available
        if not self.map.any():
            self.get_logger().info("No map available")
            return response

        # Get start and goal orientations
        current_position = Point(x=self.px, y = self.py, z=0.0)
        start_cell_y, start_cell_x = self.world_to_grid(self.map_info, current_position)

        # Get start and goal orientations
        start_orientation = request.start.pose.orientation.w
        goal_orientation = request.goal.pose.orientation.w
        
        # Find cell index of start and goal poses in map
        goal_cell_y, goal_cell_x = self.world_to_grid(self.map_info, request.goal.pose.position)

        # If start landed inside an expanded obstacle, BFS outward to find nearest free cell
        if self.safe_map[start_cell_y, start_cell_x] != 0:
            from collections import deque
            found = False
            q = deque()
            q.append((start_cell_x, start_cell_y))
            seen = set()
            seen.add((start_cell_x, start_cell_y))
            while q:
                cx, cy = q.popleft()
                if self.safe_map[cy, cx] == 0:
                    start_cell_x, start_cell_y = cx, cy
                    found = True
                    break
                for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                    nx, ny = cx+dx, cy+dy
                    if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height and (nx,ny) not in seen:
                        seen.add((nx,ny))
                        q.append((nx,ny))
            if not found:
                self.get_logger().info("Start cell in obstacle and no free cell found nearby")
                return response

        self.get_logger().info(f"Start: world({request.start.pose.position.x:.2f},{request.start.pose.position.y:.2f}) -> cell({start_cell_x},{start_cell_y}) val={self.map[start_cell_y,start_cell_x]}")
        self.get_logger().info(f"Goal:  world({request.goal.pose.position.x:.2f},{request.goal.pose.position.y:.2f}) -> cell({goal_cell_x},{goal_cell_y}) val={self.map[goal_cell_y,goal_cell_x]}")
        self.get_logger().info(f"Map shape: {self.map.shape}, free cells: {(self.map==0).sum()}")

        path = self.a_star((start_cell_x, start_cell_y, start_orientation), (goal_cell_x, goal_cell_y, goal_orientation))

        if path is None:
            self.get_logger().info("No path found")
            return response

        path = self.optimize_path(path)
        
        path_msg = self.build_path_message(path)
        if path_msg.poses:
            path_msg.poses[0].pose.position.x = self.px
            path_msg.poses[0].pose.position.y = self.py

        # Return your path 
        response.plan = path_msg # Convert the path to a Path message and set it as the response
        self.path_publisher.publish(response.plan) # Publish the path for visualization in Rviz
        return response

    def validate_trajectory(self, start_pose: Pose, goal_pose: Pose, lin_vel: float, ang_vel: float) -> bool:
        '''
        Validates a trajectory
        '''
        if self.safe_map is None or self.map_info is None:
            return False

        time_step = 0.057 / lin_vel

        curr_pose = Pose()
        curr_pose.position.x = start_pose.position.x
        curr_pose.position.y = start_pose.position.y
        curr_pose.orientation = start_pose.orientation

        goal_cell = self.world_to_grid(self.map_info, goal_pose.position)

        for _ in range(500):
            curr_cell = self.world_to_grid(self.map_info, curr_pose.position)

            # Bounds check
            if not (0 <= curr_cell[0] < self.safe_map.shape[0] and
                    0 <= curr_cell[1] < self.safe_map.shape[1]):

                self.get_logger().info(f"Invalid Trajectory: Out of bounds")
                return False

            if self.safe_map[curr_cell[0], curr_cell[1]] != 0:
                self.get_logger().info(f"Invalid Trajectory: Hits obstacle")
                return False # hit obstacle

            if curr_cell == goal_cell:
                self.get_logger().info(f"Valid Trajectory")
                return True 

            yaw = 2 * math.atan2(curr_pose.orientation.z, curr_pose.orientation.w)
            yaw += ang_vel * time_step
            curr_pose.position.x += lin_vel * math.cos(yaw) * time_step
            curr_pose.position.y += lin_vel * math.sin(yaw) * time_step
            curr_pose.orientation.z = math.sin(yaw / 2)
            curr_pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f"Invalid Trajectory: Stuck in loop")
        return False

    def optimize_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[y, x]] The path as a list of tuples (grid coordinates)
        :return     [[y, x]] The optimized path as a list of tuples (grid coordinates)
        """
        optimized_path = list(path)

        i = 1
        while i < len(optimized_path) - 1:
            prev_point = optimized_path[i - 1]
            next_point = optimized_path[i + 1] 

            prev_world = self.grid_to_world(self.map_info, prev_point)
            next_world = self.grid_to_world(self.map_info, next_point)

            prev_pose = Pose()
            prev_pose.position.x = prev_world.x
            prev_pose.position.y = prev_world.y

            next_pose = Pose()
            next_pose.position.x = next_world.x
            next_pose.position.y = next_world.y

            dx = next_world.x - prev_world.x
            dy = next_world.y - prev_world.y
            yaw = math.atan2(dy, dx)
            prev_pose.orientation.z = math.sin(yaw / 2)
            prev_pose.orientation.w = math.cos(yaw / 2)

            if self.validate_trajectory(prev_pose, next_pose, lin_vel=0.2, ang_vel=0.0):
                optimized_path.pop(i)
            else:
                i += 1

        return optimized_path 

    def neighbors_with_orientation(self,p: tuple[int, int, int]) -> list[tuple[int, int, int]]:
        """
        Returns the safe neighbours for nonholonomic robot with pose x,y,theta.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int, int)]    The coordinate in the grid.
        :return        [[(int,int, int)]]   A list of walkable 8-neighbors.
        """
        # EXTRA CREDIT
        
        neighbors = []
        x, y, theta = p

        for theta in [0, 45, 90, 135, -45, -90, -135, -180]: # Loop through the 8 directions
            new_x, new_y = round(p[0] + (math.sqrt(2) if (theta % 2) != 0 else 1) * math.cos(theta)), round(p[1] + (math.sqrt(2) if (theta % 2) == 0 else 1) * math.sin(theta))     # Calculate the new coordinates
            
            if 0 <= new_x < self.map.shape[0] and 0 <= new_y < self.map.shape[1]: # Check if coordinates are in bounds
                if self.map[new_x, new_y] == 0:      # Check if cell is walkable
                    neighbors.append((new_x, new_y, theta)) # Add valid neighbor to the list

        return neighbors


            
        



def main(args=None):
    print("Running path planner node")
    rclpy.init(args=args)
    # Add your node
    node = PathPlanner()

    executor = rclpy.executors.MultiThreadedExecutor() # Allows for simultaneous callbacks
    executor.add_node(node) #  Add a node checkpoint to the executor
    try:
        # Spin your node
        print("Path planner node is spinning")
        executor.spin() # Start loop to process callbacks

    except KeyboardInterrupt:
        pass
    finally:
        # Destroy and shutdown node when ctrl + C is pressed
        print("Shutting down path planner node")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

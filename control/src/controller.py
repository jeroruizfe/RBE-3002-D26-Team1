#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from scipy.spatial.transform import Rotation
# Transform listener imports
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs
# Callback imports
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
import numpy as np
# Math
from math import sqrt, pow, pi, atan2, sin, cos

class Controller(Node):
    def __init__(self): # Constructor for the Controller class
        # Initialize node
        super().__init__('controller')
        self.get_logger().info("Controller node has been started")

        # create a callback group for the '/odom', '/move_base_simple/goal '/nav_path' subscriptions
        self.cb_group = ReentrantCallbackGroup()

        # Subscribers and Publishers
        # Subscribe to the '/odom' topic, call self.update.odometry when messages are published to that topic
        self.sub_odom = self.create_subscription( 
            Odometry, # Type
            '/odom',  # Topic
            self.update_odometry, # Callback function (function that is called)
            10, # Buffer size 
            callback_group = self.cb_group
        )
        # Subscribe to the '/nav_path' topic that contains Path messages published by the path_generator node
        self.sub_path = self.create_subscription( 
            Path,
            '/nav_path',
            self.handle_path,
            10,
            callback_group = self.cb_group
        )
        # Subscribe to the '/move_base_simple/goal' topic that contains PoseStamped messages published by RVIZ 
        self.sub_move_base_goal = self.create_subscription( 
            PoseStamped, 
            '/move_base_simple/goal', 
            self.go_to, 
            10, 
            callback_group = self.cb_group
        )
        
        # Publish TwistStamped messages to the '/cmd_vel' topic
        self.pub_cmd_vel = self.create_publisher( 
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.pub_goal_reached = self.create_publisher(
            PoseStamped,
            '/goal_reached',
            10
        )

        self.pub_current_pose = self.create_publisher(
            PoseStamped,
            '/current_pose',
            10
        )

        self.sub_safe_map = self.create_subscription(
            OccupancyGrid,
            '/map/safe',
            self.handle_safe_map,
            10,
            callback_group=self.cb_group
        )

        # Transform listener + Transform buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
        #MSG_IN_NEW_FRAME = self.tf_buffer.transform(MSG_TO_BE_TRANSFORMED,'map', rclpy.duration.Duration(seconds=1))

        # Initialize Pose 
        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0

        self.replan = False
        self._path_seq = 0

        self.safe_map = np.zeros((0,0))

        self.validate_traj_client = self.create_client(GetPlan, 'validate_trajectory')

        self.sub_cmd_vel = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.update_actual_vel,
            10,
            callback_group=self.cb_group
        )

        self.actual_linear  = 0.0
        self.actual_angular = 0.0

        # Tests
        # self.test_drive_timer = self.create_timer(5.0, self.test_drive, callback_group=self.cb_group)
        # self.test_rotate_timer = self.create_timer(2.0, self.test_rotate)

    def call_validate_trajectory(self, start_pose, goal_pose) -> bool:
        request = GetPlan.Request()
        request.start.pose = start_pose
        request.goal.pose = goal_pose

        future = self.validate_traj_client.call_async(request)

        while not future.done():
            pass  # future will be resolved by the executor already running

        return len(future.result().plan.poses) > 0

    def handle_safe_map(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.safe_map = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )


    def send_speed_command(self, linear_speed: float, angular_speed: float):
        '''
        Sends speed to the /cmd_vel topic, which runs the turtlebot motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        '''
        
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        
        twist_msg.twist.linear.x = linear_speed
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0

        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = angular_speed
        
        self.pub_cmd_vel.publish(twist_msg)

    def drive(self, distance: float, linear_speed: float):
        '''
        Drives the robot in a straight line.

        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        '''
        #self.get_logger().info('Drive started')

        # Initial pose
        Initial_x = self.px
        Initial_y = self.py
        
        # Drive forward
        self.send_speed_command(linear_speed, 0.0)
        
        # Loop as long as ROS is running, not shutting down (Ctrl+C)
        rate = self.create_rate(10)  # Waiting at 10 Hz
        while rclpy.ok():
            # Calculate distance: (current - initial) = distance traveled
            distance_traveled = sqrt( pow(self.px - Initial_x, 2) + pow(self.py - Initial_y, 2) )
            distance_error = distance - distance_traveled

            # If within 1 cm of the target distance
            if distance_error <= 0.01:
                self.send_speed_command(0.0, 0.0)  # Stop the robot
                break

            rate.sleep()

        #self.get_logger().info('Drive completed')

    def rotate(self, angle: float, angular_speed: float):
        '''
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        '''
        self.get_logger().info(f"Rotation started in angle {angle}")

        # Initial orientation
        Initial_th = self.pth

        # If angle is negative rotate clockwise, positive is counterclockwise
        if angle < 0:
            angular_speed = -angular_speed

        # Rotate
        self.send_speed_command(0.0, angular_speed)

        # Loop as long as ROS is running, not shutting down (Ctrl+C)
        rate = self.create_rate(10)  # Waiting at 10 Hz
        while rclpy.ok():
            angle_difference = self.pth - Initial_th # Angle difference between current and initial orientation
            angle_error = angle - angle_difference

            # Finds the shortest angle to turn
            if (angle_error > pi):
                angle_error -= 2*pi
            elif (angle_error < -pi):
                angle_error += 2*pi

            # If within 0.02 rad of the target angle
            if abs(angle_error) <= 0.02:
                self.send_speed_command(0.0, 0.0)  # Stop the robot
                break

            rate.sleep()

        #self.get_logger().info('Rotation completed')

    # ------------------------------------------------------------------
    # go_to function using protortional and derivative control
    # ------------------------------------------------------------------

    def go_to(self, msg: PoseStamped, path_seq: int = -1):
        '''
        :param msg [PoseStamped] The target or "goal" pose.
        :param path_seq [int] The sequence number of the path.
        '''

        # Transform given pose to odom frame
        msg.header.stamp = rclpy.time.Time().to_msg()
        pose_odom = self.tf_buffer.transform(msg, 'odom', rclpy.duration.Duration(seconds=1))

        # Final pose (odom)
        Final_x = pose_odom.pose.position.x
        Final_y = pose_odom.pose.position.y
        Final_orig = pose_odom.pose.orientation

        # kp constants
        # kp_linear_speed =  0.36 #9
        # kp_angular_speed = 0.42 #7

        # # kd constants
        # kd_linear_speed = 0.12
        # kd_angular_speed = 0.19

        kp_linear_speed = 0.25      # 0.25
        kd_linear_speed = 0.08      # 0.08

        kp_angular_speed = 0.74     # 0.08
        kd_angular_speed = 0.09     # 0.09

        derrivative_control = False # Becomes true after calculating the first error

        rate = self.create_rate(10)

        while rclpy.ok():
            if path_seq >= 0 and self._path_seq != path_seq:
                break

            time = self.get_clock().now()
            heading = atan2(Final_y - self.py, Final_x - self.px)
            heading_error = heading - self.pth
            if (heading_error > pi): heading_error -= 2*pi
            elif (heading_error < -pi): heading_error += 2*pi
            distance_error = sqrt(pow(Final_x - self.px, 2) + pow(Final_y - self.py, 2))
            if distance_error < 0.02: break

            # Don't drive forward until roughly facing the goal
            if abs(heading_error) > 0.3:
                self.send_speed_command(0.0, heading_error * kp_angular_speed)
                rate.sleep()
                continue

            # Distance: distance between current position and target position
            distance_error = sqrt(pow(Final_x - self.px, 2) + pow(Final_y - self.py, 2))

            # Only move if distance is further than a 0.02 tolerance            
            # Calculate efforts
            if derrivative_control and (time - previous_time).nanoseconds / 1e9 > 0:
                # Calculate change in time
                dt = (time - previous_time).nanoseconds / 1e9

                # Calculate linear effort
                derivative_linear = (distance_error - previous_distance_error) / dt # Calculate derrivative portion of linear effort formula
                linear_effort = distance_error * kp_linear_speed + derivative_linear * kd_linear_speed

                # Calculate angular effort
                derivative_angular = (heading_error - previous_heading_error) / dt # Calculate derrivative portion of angular effort formula
                angular_effort = heading_error * kp_angular_speed + derivative_angular * kd_angular_speed

            else:
                linear_effort = (distance_error) * kp_linear_speed
                angular_effort = (heading_error) * kp_angular_speed

            previous_distance_error = distance_error
            previous_heading_error = heading_error

            # Command Speeds            
            if linear_effort < 0.05: linear_effort = 0.05

            self.send_speed_command(linear_effort, angular_effort)

            derrivative_control = True # Change to false to disable derrivative control

            previous_time = time
            rate.sleep()
            
        self.send_speed_command(0.0, 0.0)

        self.get_logger().info('Destination reached')

    # ------------------------------------------------------------------
    # go_to function using rotate and drive fuctions
    # ------------------------------------------------------------------

    # def go_to(self, msg: PoseStamped):
    #     '''
    #     Uses rotate() and drive() to get to a specific pose.
    #     :param msg [PoseStamped] The target or "goal" pose.
    #     '''

    #     # Transform given pose to odom frame
    #     msg.header.stamp = rclpy.time.Time().to_msg()
    #     pose_odom = self.tf_buffer.transform(msg, 'odom', rclpy.duration.Duration(seconds=1))


    #     # Final pose (odom)
    #     Final_x = pose_odom.pose.position.x
    #     Final_y = pose_odom.pose.position.y
    #     Final_orig = pose_odom.pose.orientation

    #     # Heading: angle between current position and target position
    #     heading = atan2(Final_y - self.py, Final_x - self.px) # Angle to face to go to the target position
    #     heading_difference = heading - self.pth # Angle difference between current heading and target heading

    #     # Finds the shortest angle to turn
    #     if (heading_difference > pi):
    #         heading_difference -= 2*pi
    #     elif (heading_difference < -pi):
    #         heading_difference += 2*pi

    #     # Distance: distance between current position and target position
    #     distance_error = sqrt( pow(Final_x - self.px, 2) + pow(Final_y - self.py, 2) )

    #     # Only move if distance is further than a 0.02 tolerance
    #     if distance_error > 0.02:
    #         # Rotate
    #         self.rotate(heading_difference, 0.2)

    #         # Drive
    #         # self.drive(distance_error, 0.2)
    #         self.smooth_drive(distance_error, 0.2)

    def update_odometry(self, msg: Odometry):
        '''
        A callback that updates the current pose of the robot
        :param msg [Odometry] the current odometry information
        '''
        # Current pose (odom)
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        (roll, pitch, yaw) = Rotation.from_quat([quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]).as_euler('xyz')
        self.pth = yaw
        self.publish_current_pose()

    def update_actual_vel(self, msg: TwistStamped):
        self.actual_linear  = msg.twist.linear.x
        self.actual_angular = msg.twist.angular.z

    def publish_current_pose(self):
        current_pose_msg = PoseStamped()
        current_pose_msg.header.stamp = rclpy.time.Time().to_msg()  # ← was: self.get_clock().now().to_msg()
        current_pose_msg.header.frame_id = 'odom'
        current_pose_msg.pose.position.x = self.px
        current_pose_msg.pose.position.y = self.py
        current_pose_msg.pose.position.z = 0.0
        quat = Rotation.from_euler('z', self.pth).as_quat()
        current_pose_msg.pose.orientation.x = quat[0]
        current_pose_msg.pose.orientation.y = quat[1]
        current_pose_msg.pose.orientation.z = quat[2]
        current_pose_msg.pose.orientation.w = quat[3]

        try:
            pose_map = self.tf_buffer.transform(current_pose_msg, 'map', rclpy.duration.Duration(seconds=0))
            self.pub_current_pose.publish(pose_map)
        except Exception as e:
            self.get_logger().warn(f'Could not transform pose to map frame: {e}')
            self.pub_current_pose.publish(current_pose_msg)  # fall back to odom frame
  
    def test_drive(self): # Test drive function
        self.test_drive_timer.cancel() # Cancel after 1 run
        self.drive(1.0, 0.5) # Drive forward 1 meter at 0.5 m/s

    def test_rotate(self): # Test rotate function
        self.test_rotate_timer.cancel() # Cancel after 1 run
        self.rotate(pi/2, 0.5) # Rotate 90 degrees at 0.5 rad/s

    def handle_path(self, path: Path):
        self.get_logger().info('path is being handled')
        self._path_seq += 1
        my_seq = self._path_seq
        self.replan = False

        for point in path.poses:
            if self._path_seq != my_seq:
                self.get_logger().info('Replanning triggered, abandoning current path')
                return

            # Build start_pose from current odometry
            start_pose = Pose()
            start_pose.position.x = self.px
            start_pose.position.y = self.py
            dx = point.pose.position.x - self.px
            dy = point.pose.position.y - self.py
            yaw = atan2(dy, dx)
            start_pose.orientation.z = sin(yaw / 2)
            start_pose.orientation.w = cos(yaw / 2)

            if self.call_validate_trajectory(start_pose, point.pose):
                self.get_logger().info('Using pure pursuit')
                self.pure_pursuit(point, my_seq)
            else:
                self.get_logger().info('Using go_to')
                self.go_to(point, my_seq)

            if self._path_seq != my_seq:
                self.get_logger().info('Replanning triggered, abandoning current path')
                return

        self.pub_goal_reached.publish(PoseStamped())
        self.get_logger().info('Final destination reached')


    def pure_pursuit(self, msg: PoseStamped, path_seq: int = -1):
        msg.header.stamp = rclpy.time.Time().to_msg()

        pose_odom = self.tf_buffer.transform(msg, 'odom', rclpy.duration.Duration(seconds=1))

        Final_x = pose_odom.pose.position.x
        Final_y = pose_odom.pose.position.y

        # Geometry constants
        LOOKAHEAD = 0.45    # meters
        MAX_LINEAR = 0.15   # m/s

        # PID constants — tune these
        kp_lin, ki_lin, kd_lin = 0.003, 0.0, 0.0
        kp_ang, ki_ang, kd_ang = 0.003, 0.0, 0.0

    

        # PID state
        integral_lin = integral_ang = 0.0
        prev_error_lin = prev_error_ang = 0.0
        prev_time = None

        rate = self.create_rate(10)

        while rclpy.ok():
            if path_seq >= 0 and self._path_seq != path_seq:
                self.send_speed_command(0.0, 0.0)
                return

            now = self.get_clock().now()
            dt = (now - prev_time).nanoseconds / 1e9 if prev_time is not None else 0.1
            prev_time = now

            # --- Circle geometry: compute target velocities ---
            dx = Final_x - self.px
            dy = Final_y - self.py
            distance = sqrt(dx**2 + dy**2)

            if distance < 0.35:
                break

            # Transform goal to robot local frame
            local_x =  cos(self.pth) * dx + sin(self.pth) * dy
            local_y = -sin(self.pth) * dx + cos(self.pth) * dy

            ld = min(LOOKAHEAD, distance)
            scale = ld / distance
            lx = local_x * scale
            ly = local_y * scale

            # Curvature from circle geometry: k = 2*ly / ld^2
            curvature = (2.0 * ly) / (ld ** 2)

            # Target velocities from geometry
            alignment = lx / ld  # 1.0 = facing goal, 0.0 = 90° off

            # Scale linear down when misaligned but never fully to zero
            # Use a softer scaling so robot moves while turning
            target_linear = MAX_LINEAR * max(alignment, 0.1)  # minimum 0.1 * MAX_LINEAR

            # Always turn toward goal regardless of alignment
            target_angular = target_linear * curvature  # angular independent of linear

            # But do stop linear if facing completely wrong way (> 90°)
            if alignment >= 0.0 and alignment < 0.75 :
                target_linear = 0.0
            

            # --- PID on velocity error ---
            error_lin = target_linear  - self.actual_linear
            error_ang = target_angular - self.actual_angular

            # Integral with anti-windup
            integral_lin = max(min(integral_lin + error_lin * dt, 0.3), -0.3)
            integral_ang = max(min(integral_ang + error_ang * dt, 2.0), -2.0)

            # Derivative
            d_lin = (error_lin - prev_error_lin) / dt if dt > 0 else 0.0
            d_ang = (error_ang - prev_error_ang) / dt if dt > 0 else 0.0

            # PID output = correction to add on top of target
            cmd_linear  = target_linear  + kp_lin * error_lin 
            # + ki_lin * integral_lin + kd_lin * d_lin
            cmd_angular = target_angular + kp_ang * error_ang 
            # + ki_ang * integral_ang + kd_ang * d_ang

            # Clamp outputs
            cmd_linear  = max(min(cmd_linear,  MAX_LINEAR), 0.0)
            cmd_angular = max(min(cmd_angular, 1.0), -1.0)

            # Stop driving forward if badly misaligned
            if abs(local_y / ld) > sin(np.pi / 2):  # more than 90° off in local frame
                cmd_linear = 0.0

            self.get_logger().info(
                f'dist={distance:.3f} target_v={target_linear:.3f} target_w={target_angular:.3f} '
                f'err_v={error_lin:.3f} err_w={error_ang:.3f} '
                f'cmd_v={cmd_linear:.3f} cmd_w={cmd_angular:.3f}'
            )

            self.send_speed_command(cmd_linear, cmd_angular)

            prev_error_lin = error_lin
            prev_error_ang = error_ang

            rate.sleep()

        self.send_speed_command(0.0, 0.0)
        self.get_logger().info('Destination reached')
            

    # EXTRA CREDIT
    def smooth_drive(self, distance: float, linear_speed: float):
        '''
        Smoothly drives the robot in a straight line by regulating its speed.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        '''
        # Initial pose
        Initial_x = self.px
        Initial_y = self.py
        
        # Loop as long as ROS is running, not shutting down (Ctrl+C)
        rate = self.create_rate(10)  # Waiting at 10 Hz
        while rclpy.ok():
            # Calculate distance: (current - initial) = distance traveled
            distance_traveled = sqrt( pow(self.px - Initial_x, 2) + pow(self.py - Initial_y, 2) )
            distance_error = distance - distance_traveled

            progress = distance_traveled / distance # Progress is 0 to 1
            speed = linear_speed * sin(progress * pi) # Follows sine curve, low at start and end while high in the middle
            speed = max(0.1, speed) # Minimum speed of 0.1 m/s to prevent stalling

            self.send_speed_command(speed, 0.0)

            # If within 1 cm of the target distance
            if distance_error <= 0.01:
                self.send_speed_command(0.0, 0.0)  # Stop the robot
                break

            rate.sleep()





def main(args=None):
    print("Running controller node")
    rclpy.init(args=args)
    node = Controller()

    # Node callback 
    executor = rclpy.executors.MultiThreadedExecutor() # Allows for simultaneous callbacks
    executor.add_node(node) #  Add a node checkpoint to the executor
    try:
        print("Controller node is spinning")
        executor.spin() # Start loop to process callbacks
    finally:
        print("Shutting down controller node")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

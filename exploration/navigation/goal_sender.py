#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan
from rclpy.callback_groups import ReentrantCallbackGroup


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.cb_group = ReentrantCallbackGroup()

        self.path_client = self.create_client(
            GetPlan,
            '/request_plan',
            callback_group=self.cb_group
        )

        # Current pose — updated continuously from AMCL so every new goal
        # uses the real current position, not the one-shot localized pose.
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'map'

        # Track whether we are currently executing a goal to avoid
        # sending a second plan before the first one finishes.
        self.navigating = False

        # Subscribe to AMCL pose for continuous position updates.
        # This replaces the one-shot /current_pose/localized subscription
        # so that each new goal starts from the actual current position.
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10,
            callback_group=self.cb_group
        )

        # Keep the one-shot subscription so we know when the robot is
        # first localized and safe to accept goals.
        self.localized = False
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose/localized',
            self.localized_callback,
            10,
            callback_group=self.cb_group
        )

        # Accept goal from RViz "2D Goal Pose" button.
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10,
            callback_group=self.cb_group
        )

        # Wait for path planner service before accepting goals.
        self.ready = False
        self.readiness_timer = self.create_timer(
            1.0, self.check_ready, callback_group=self.cb_group
        )

        self.get_logger().info('GoalSender ready. Waiting for path planner service...')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def check_ready(self):
        if self.path_client.service_is_ready():
            self.ready = True
            self.readiness_timer.cancel()
            self.get_logger().info(
                'Path planner available. Send a 2D Goal Pose in RViz to navigate.'
            )

    def localized_callback(self, msg: PoseStamped):
        """One-shot: marks the robot as localized so goals are accepted."""
        if not self.localized:
            self.localized = True
            self.current_pose = msg
            self.get_logger().info('Robot localized — goals will now be accepted.')

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Continuously update current pose from AMCL so multi-goal works correctly."""
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def goal_callback(self, msg: PoseStamped):
        if not self.ready:
            self.get_logger().warn('Goal received but path planner not ready yet — ignoring.')
            return

        if not self.localized:
            self.get_logger().warn('Goal received but robot is not yet localized — ignoring.')
            return

        if self.navigating:
            self.get_logger().warn(
                'Goal received but robot is still navigating to previous goal — ignoring. '
                'Wait for the current goal to complete before sending a new one.'
            )
            return

        self.get_logger().info(
            f'Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

        msg.header.frame_id = 'map'

        req = GetPlan.Request()
        req.start = self.current_pose
        req.start.header.frame_id = 'map'
        req.goal = msg
        req.tolerance = 0.1

        self.navigating = True
        future = self.path_client.call_async(req)
        future.add_done_callback(self.path_response_callback)

    def path_response_callback(self, future):
        try:
            result = future.result()
            if result.plan.poses:
                self.get_logger().info(
                    f'Path planned with {len(result.plan.poses)} waypoints — robot navigating.'
                )
                # navigating stays True until the controller signals completion.
                # Subscribe to the done signal published by the controller/path_planner.
                self._done_sub = self.create_subscription(
                    PoseStamped,
                    '/goal_reached',
                    self._goal_reached_callback,
                    1,
                    callback_group=self.cb_group
                )
            else:
                self.get_logger().warn('Path planner returned empty path — is the goal reachable?')
                self.navigating = False  # Allow a new goal attempt
        except Exception as e:
            self.get_logger().error(f'Path planning failed: {e}')
            self.navigating = False  # Allow retry

    def _goal_reached_callback(self, msg: PoseStamped):
        """Called when the controller publishes on /goal_reached."""
        self.get_logger().info(
            'Goal reached! Ready for next goal — send a 2D Goal Pose in RViz.'
        )
        self.navigating = False
        # Unregister so we don't fire again until the next goal.
        self.destroy_subscription(self._done_sub)


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
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
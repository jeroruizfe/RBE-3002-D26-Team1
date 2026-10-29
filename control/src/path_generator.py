#!/usr/bin/env python3

import rclpy

from typing import Tuple, List
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathGenerator(Node):
    def __init__(self, px, py, pth, frame):
        super().__init__('path_generator')

        self.px = px
        self.py = py
        self.pth = pth
        self.frame = frame

        # note the topic that the path is being published to
        self.send_path = self.create_publisher(Path, '/nav_path', 10)
        self.get_logger().info('test path node started')

        self.generate_path()

    def convert_to_nav_msg(self, path: List[Tuple]):
        new_path = []
        path_message = Path()

        for position in path:
            pose_message = PoseStamped()
            pose_message.header.stamp = self.get_clock().now().to_msg()
            pose_message.header.frame_id = self.frame
            pose_message.pose.position.x = float(position[0])
            pose_message.pose.position.y = float(position[1])
            new_path.append(pose_message)
        
        path_message.header.stamp = self.get_clock().now().to_msg()
        path_message.header.frame_id = self.frame
        path_message.poses = new_path

        return path_message

    def generate_path(self):
        path_array = [(0.0, 0), 
                      (0.03,0.0),
                      (0.06,0.0),
                      (0.09,0.03),
                      (0.12,0.06),
                      (0.15,0.06),
                      (0.18,0.06),
                      (0.21,0.06),
                      (0.24,0.06),
                      (0.24,0.06),
                      (0.27,0.03),
                      (0.30,0.0),
                      (0.33,0.0),
                      (0.36,0.0),
                      (0.36,-0.03),
                      (0.36,-0.06),
                      (0.36,-0.09),
                      (0.36,-0.12),
                      (0.36,-0.15),
                      (0.36,-0.18),
                      (0.36,-0.21),
                      (0.36,-0.24),
                      (0.36,-0.27),
                      (0.36,-0.30)]
        path_of_poses = self.convert_to_nav_msg(path_array)
        self.send_path.publish(path_of_poses)
        self.get_logger().info('published path!')

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator(0,0,0,'odom')
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown() 
   
if __name__ == '__main__':
    main()

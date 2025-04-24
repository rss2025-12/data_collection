import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
import tf_transformations

import sys, os, csv, time
from .utils import LineTrajectory

import numpy as np


class Lab6Path(Node):
    def __init__(self):
        super().__init__('lab_6_path')

        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                 "/initialpose",
                                                 1)
        self.goal_pub = self.create_publisher(PoseStamped,
                                              "/goal_pose",
                                              1)
        self.pose_pub = self.create_publisher(Pose, '/pose', 10)

        self.trajectory = LineTrajectory(node=self, viz_namespace="/data_trajectory")

        self.tests = {
            'short': [(0.0, 0.0), (-15.0, 12.0)],
            'medium': [(0.0, 0.0), (-20.0, 34.0)],
            'long': [(0.0, 0.0), (-55.0, 35.0)],
            'real': [(-16.0, 10.0), (-5.5, 25.0)]
            }

        self.test = 'long'
        self.algoithm = 'rrt'
        self.cross_track = False

        if self.cross_track is False:
            output_path = os.path.join(os.path.dirname(__file__), '../../data/lab_6/rrt_long.csv')
            os.makedirs(os.path.dirname(output_path), exist_ok=True)

            self.csv_file = open(output_path, mode='a', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            # Only write the header if the file is empty
            if os.stat(output_path).st_size == 0:
                self.csv_writer.writerow(['timestamp', 'algorithm', 'test', 'time', 'length'])

        self.set_start_and_end(self.test)

    def set_start_and_end(self, test=None):
        start_x, start_y = self.tests[self.test][0]
        goal_x, goal_y = self.tests[self.test][1]

        # Set initial pose
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'

        initial_pose_msg.pose.pose.position.x = start_x
        initial_pose_msg.pose.pose.position.y = start_y
        initial_pose_msg.pose.pose.position.z = 0.0

        # Orientation (pi radians, facing backwards along the x-axis)
        # Quaternion for a 180-degree rotation (π radians) around the Z-axis
        initial_pose_msg.pose.pose.orientation.x = 0.0
        initial_pose_msg.pose.pose.orientation.y = 0.0
        initial_pose_msg.pose.pose.orientation.z = np.sin(np.pi / 2)
        initial_pose_msg.pose.pose.orientation.w = np.cos(np.pi/ 2)

        # Publish goal pose
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y

        self.initial_pub.publish(initial_pose_msg)
        self.goal_pub.publish(goal_msg)

        self.start_time = time.time()
        self.get_logger().info("Starting test")

    def trajectory_callback(self, msg):
        if self.cross_track is False:
            elapsed_time = time.time() - self.start_time

            self.trajectory.clear()
            self.trajectory.fromPoseArray(msg)
            length = self.trajectory.distances[-1]
            self.get_logger().info(f"Recieved path in {elapsed_time}")
            self.get_logger().info(f"Total trajectory length = {self.trajectory.distances[-1]} meters")

            timestamp = self.get_clock().now().to_msg().sec
            self.csv_writer.writerow([timestamp, self.algoithm, self.test, elapsed_time, length])

def main(args=None):
    rclpy.init(args=args)
    node = Lab6Path()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Optional: print("KeyboardInterrupt received")
    finally:
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

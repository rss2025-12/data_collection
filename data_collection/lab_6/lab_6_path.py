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

        self.trajectory = LineTrajectory(self)

        self.tests = {
            'short': [(0.0, 0.0), (-15.0, 12.0)],
            'medium': [(0.0, 0.0), (-20.0, 34.0)],
            'long': [(0.0, 0.0), (-55.0, 35.0)],
            'real': [(-16.0, 10.0), (-5.5, 25.0)]
            }

        self.testing_time = False
        self.num_tests = 30
        self.run_tests = 0
        self.test = 'short'
        self.test_ready = True

        if self.testing_time:
            output_path = os.path.join(os.path.dirname(__file__), '../../data/lab_6/path_algorithm.csv')
            os.makedirs(os.path.dirname(output_path), exist_ok=True)

            self.csv_file = open(output_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'test', 'time'])
            self.timer = self.create_timer(0.1, self.set_start_and_end)
        else:
            output_path = os.path.join(os.path.dirname(__file__), '../../data/lab_6/dummy.csv')
            os.makedirs(os.path.dirname(output_path), exist_ok=True)

            self.csv_file = open(output_path, mode='w', newline='')
            self.set_start_and_end('real') # Set test case here

        self.get_logger().info("Testing path planning algorithm")

    def set_start_and_end(self, test=None):
        if self.run_tests == 10:
            self.test = 'medium'
        elif self.run_tests == 20:
            self.test = 'long'
        elif self.run_tests >= self.num_tests:
            self.get_logger().info("Tests complete")
            if self.testing_time:
                self.csv_file.close()
            return

        if not self.test_ready:
            return

        if test is not None:
            start_x, start_y = self.tests[test][0]
            goal_x, goal_y = self.tests[test][1]
        else:
            start_x, start_y = self.tests[self.test][0]
            goal_x, goal_y = self.tests[self.test][1]

        x = 0.0
        y = 0.0
        theta = np.pi

        # Set robot pose
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y

        quaternion = tf_transformations.quaternion_from_euler(0, 0, theta)
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]

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
        initial_pose_msg.pose.pose.orientation.z = np.sin(theta / 2)
        initial_pose_msg.pose.pose.orientation.w = np.cos(theta / 2)

        for _ in range(10):
            self.pose_pub.publish(pose_msg)
            self.initial_pub.publish(initial_pose_msg)

        # Publish goal pose
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        self.goal_pub.publish(goal_msg)

        self.start_time = time.time()
        self.test_ready = False

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory with {len(msg.poses)} points")

        if self.testing_time:
            elapsed_time = time.time() - self.start_time
            timestamp = self.get_clock().now().to_msg().sec
            self.csv_writer.writerow([timestamp, self.test, elapsed_time])
            self.run_tests += 1
            self.test_ready = True
        else:
            self.trajectory.clear()
            self.trajectory.fromPoseArray(msg)
            self.trajectory.publish_viz(duration=0.0)


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

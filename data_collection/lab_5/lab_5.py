import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import os, csv

class Lab5(Node):
    def __init__(self):
        super().__init__('lab_5')

        # Ground truth
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Estimated pose
        self.estimated_pose_sub = self.create_subscription(
            Odometry,
            '/pf/pose/odom',
            self.pose_callback,
            10
        )

        self.ground_truth = None
        self.estimated_pose = None

        # Writing data
        output_path = os.path.join(os.path.dirname(__file__), '../../data/lab_5/test_convergence_.1_.05.csv') # File name
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        self.csv_file = open(output_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'position_error', 'yaw_error', 'cross_track_error']) # Header

        # Timer to periodically compute error
        self.timer = self.create_timer(1/5, self.compute_error)
        self.t0 = -1


    def get_ground_truth_pose(self):
        trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        translation = trans.transform.translation
        rotation = trans.transform.rotation
        euler = euler_from_quaternion([
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        ])
        return [translation.x, translation.y, euler[2]]


    def pose_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        euler = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        self.estimated_pose = [position.x, position.y, euler[2]]


    def compute_error(self):
        self.ground_truth = self.get_ground_truth_pose()
        if self.estimated_pose is None:
            return

        gt = np.array(self.ground_truth)
        est = np.array(self.estimated_pose)

        # Position error (Euclidean distance)
        position_error = np.linalg.norm(gt[:2] - est[:2])

        # Yaw error
        yaw_diff = est[2] - gt[2]
        yaw_error = (yaw_diff + np.pi) % (2 * np.pi) - np.pi  # Normalize

        # Cross-track error
        # get current rover position, corss track error is error along y axis
        theta = gt[2]
        cross_track_error = float((gt[:2] - est[:2]).reshape((1,2))@np.array([[np.sin(theta)],[np.cos(theta)]]))

        # Log to CSV
        timestamp = self.get_clock().now().nanoseconds / 1e9
        if self.t0 == -1: self.t0 = timestamp
        timestamp -= self.t0
        self.csv_writer.writerow([timestamp, position_error, yaw_error, cross_track_error])


def main(args=None):
    rclpy.init(args=args)
    node = Lab5()
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

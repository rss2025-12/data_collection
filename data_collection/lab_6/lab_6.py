import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import tf_transformations

import sys, os, csv
sys.path.append(os.path.abspath('../../../path_planning/path_planning'))
from .utils import LineTrajectory

import numpy as np


class Lab6(Node):
    def __init__(self):
        super().__init__('lab_6')

        self.odom_sub = self.create_subscription(Odometry,
                                                 '/odom',
                                                 self.pose_callback,
                                                 1)
        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               '/drive',
                                               10)
        self.pose_pub = self.create_publisher(Pose,
                                              '/pose',
                                              10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      '/initialpose',
                                                      10)
        self.goal_pub = self.create_publisher(PoseStamped,
                                              "/goal_pose",
                                              1)

        # # Stop robot
        # drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = self.get_clock().now().to_msg()
        # drive_msg.drive.speed = 0.0
        # drive_msg.drive.steering_angle = 0.0
        # self.drive_pub.publish(drive_msg)

        self.initialized_traj = True
        self.trajectory = LineTrajectory(node=self, viz_namespace="/data_trajectory")

        self.tests = {
            'short': [(0.0, 0.0), (-15.0, 12.0)],
            'medium': [(0.0, 0.0), (-20.0, 34.0)],
            'long': [(0.0, 0.0), (-55.0, 35.0)],
            'real': [(-16.0, 10.0), (-5.5, 25.0)]
            }
        self.test = 'long'

        # Writing data
        self.write = True
        if self.write is True:
            output_path = os.path.join(os.path.dirname(__file__), '../../data/lab_6/pf_cross_track_long_new.csv') # File name
            os.makedirs(os.path.dirname(output_path), exist_ok=True)

            self.csv_file = open(output_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'test', 'cross_track error'])
        else:
            output_path = os.path.join(os.path.dirname(__file__), '../../data/lab_6/dummy.csv')
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            self.csv_file = open(output_path, mode='w', newline='')

        self.set_start_and_end()

    def set_start_and_end(self):
        start_x, start_y = self.tests[self.test][0]
        theta = np.pi
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
        initial_pose_msg.pose.pose.orientation.z = np.sin(theta / 2)
        initial_pose_msg.pose.pose.orientation.w = np.cos(theta / 2)

        # Publish goal pose
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y

        self.initial_pub.publish(initial_pose_msg)
        self.goal_pub.publish(goal_msg)

        self.get_logger().info("Set pose and sent trajectory")

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory with {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        start_x, start_y = self.tests[self.test][0]
        theta = np.pi

        # Set robot pose
        pose_msg = Pose()
        pose_msg.position.x = start_x
        pose_msg.position.y = start_y

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

        self.initial_pub.publish(initial_pose_msg)
        self.pose_pub.publish(pose_msg)

        self.initialized_traj = True

    def pose_callback(self, msg):
        if self.trajectory is False:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        crosstrack_error = self.compute_cte(self.trajectory.points, (x, y))

        timestamp = self.get_clock().now().nanoseconds / 1e9

        if self.write is True:
            self.csv_writer.writerow([timestamp, self.test, crosstrack_error])

    def compute_cte(self, trajectory, position):
        """
        trajectory: list of (x, y) tuples
        position: (x, y) tuple of current car position
        returns: crosstrack error (signed)
        """
        px, py = position
        min_dist = float('inf')
        cte = 0.0

        for i in range(len(trajectory) - 1):
            x1, y1 = trajectory[i]
            x2, y2 = trajectory[i + 1]

            # Vector from segment start to end
            dx = x2 - x1
            dy = y2 - y1

            # Vector from segment start to current position
            dx1 = px - x1
            dy1 = py - y1

            # Project point onto segment
            seg_len_squared = dx**2 + dy**2
            if seg_len_squared == 0:
                # Degenerate segment
                proj_x, proj_y = x1, y1
            else:
                t = max(0, min(1, (dx * dx1 + dy * dy1) / seg_len_squared))
                proj_x = x1 + t * dx
                proj_y = y1 + t * dy

            # Distance from position to closest point on segment
            dist = np.hypot(px - proj_x, py - proj_y)

            if dist < min_dist:
                min_dist = dist
                # Determine sign of CTE using the cross product
                cross = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
                cte = dist if cross > 0 else -dist

        return cte

def main(args=None):
    rclpy.init(args=args)
    node = Lab6()
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

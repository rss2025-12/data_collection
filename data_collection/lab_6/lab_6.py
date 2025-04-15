import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

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

        self.initialized_traj = False
        self.trajectory = LineTrajectory(self)

        # Writing data
        output_path = os.path.join(os.path.dirname(__file__), '../../data/lab_6/test.csv') # File name
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        self.csv_file = open(output_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'cross_track error']) # Header

        self.get_logger().info("Data collection started")

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory with {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.initialized_traj = True

    def pose_callback(self, msg):
        if self.initialized_traj is False:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        crosstrack_error = self.compute_crosstrack_error(self.trajectory.points, (x, y))

        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.csv_writer.writerow([timestamp, crosstrack_error])

    def compute_crosstrack_error(self, trajectory, position):
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

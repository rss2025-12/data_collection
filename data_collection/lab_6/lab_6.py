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

    def compute_cte_vectorized(trajectory, position):
        """
        Vectorized cross-track error calculation.

        Parameters:
            trajectory: list of (x, y) waypoints
            position: (x, y) car position

        Returns:
            signed cross-track error (float)
        """
        traj = np.array(trajectory)
        pos = np.array(position)

        p1 = traj[:-1]   # start of each segment
        p2 = traj[1:]    # end of each segment
        seg_vec = p2 - p1           # segment vectors
        pt_vec = pos - p1           # vector from segment start to position

        seg_len_sq = np.sum(seg_vec**2, axis=1)
        seg_len_sq[seg_len_sq == 0] = 1e-6  # avoid divide-by-zero

        # Projection factor t of position onto each segment
        t = np.clip(np.sum(pt_vec * seg_vec, axis=1) / seg_len_sq, 0.0, 1.0)

        # Closest points on each segment
        proj = p1 + seg_vec * t[:, np.newaxis]

        # Distances to projections
        diff = pos - proj
        dists = np.linalg.norm(diff, axis=1)

        # Find closest segment
        min_idx = np.argmin(dists)
        cte = dists[min_idx]

        # Determine sign using 2D cross product
        v1 = seg_vec[min_idx]
        v2 = pos - p1[min_idx]
        cross = v1[0]*v2[1] - v1[1]*v2[0]
        signed_cte = cte if cross > 0 else -cte

        return signed_cte

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

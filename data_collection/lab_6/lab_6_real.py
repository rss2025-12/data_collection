import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import csv
import os


class Lab6Real(Node):
    def __init__(self):
        super().__init__('cross_track_error_node')
        self.path = None
        self.file_name = '../../data/lab_6/cte_real.csv'

        # Create CSV and write header if not exists
        output_path = os.path.join(os.path.dirname(__file__), self.file_name)
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        self.csv_file = open(output_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'robot_x', 'robot_y', 'cross_track error'])

        self.marker_sub = self.create_subscription(Marker, '/planned_trajectory/path', self.marker_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/pf/pose/odom', self.odom_callback, 10)

        self.path_points = None

    def marker_callback(self, msg: Marker):
        if msg.type == Marker.LINE_STRIP:
            self.path_points = [(p.x, p.y) for p in msg.points]
            self.get_logger().info(f"Received marker with {len(self.path_points)} path points.")

    def odom_callback(self, msg: Odometry):
        if self.path_points is None or len(self.path_points) < 2:
            return  # Need at least 2 points to form a segment

        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        robot_pos = np.array([robot_x, robot_y])

        # Compute the minimum distance from robot to any segment
        min_dist = float('inf')
        for i in range(len(self.path_points) - 1):
            p1 = np.array(self.path_points[i])
            p2 = np.array(self.path_points[i + 1])
            dist = self.point_to_segment_distance(robot_pos, p1, p2)
            if dist < min_dist:
                min_dist = dist

        crosstrack_error = min_dist

        # Log with ROS time
        now = self.get_clock().now().to_msg()
        timestamp = now.sec + now.nanosec * 1e-9

        # Write to CSV
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.csv_writer.writerow([timestamp, robot_x, robot_y, crosstrack_error])

    @staticmethod
    def point_to_segment_distance(p, a, b):
        """Returns the perpendicular distance from point p to segment ab."""
        ap = p - a
        ab = b - a
        ab_len_squared = np.dot(ab, ab)
        if ab_len_squared == 0.0:
            return np.linalg.norm(ap)  # a == b
        t = max(0, min(1, np.dot(ap, ab) / ab_len_squared))
        projection = a + t * ab
        return np.linalg.norm(p - projection)

def main(args=None):
    rclpy.init(args=args)
    node = Lab6Real()
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

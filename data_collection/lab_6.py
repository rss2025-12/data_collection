import rclpy
from rclpy.node import Node

import numpy as np
import os, csv

class Lab6(Node):
    def __init__(self):
        super().__init__('lab_6')


def main(args=None):
    rclpy.init(args=args)
    node = Lab6()
    try:
        rclpy.spin(node)
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

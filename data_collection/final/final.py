import rclpy
from rclpy.node import Node

import numpy as np
import os, csv

class Final(Node):
    def __init__(self):
        super().__init__('final')


def main(args=None):
    rclpy.init(args=args)
    node = Final()
    try:
        rclpy.spin(node)
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

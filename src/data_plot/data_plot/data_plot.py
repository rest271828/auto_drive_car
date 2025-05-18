import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

import rclpy.logging

logging = rclpy.logging.get_logger('test_logger')

class PlotData(Node):
    def __init__(self):
        super().__init__('data_plot_node')
        self.get_logger().info("Data Plot_Node Initialized created")

def main():
    rclpy.init()
    plot_node  = PlotData()

    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        logging.warn("Keyboard interrupt detected. Exiting...")
    finally:
        rclpy.shutdown()        #防止ctrl+c退出时，节点没有关闭

if __name__ == '__main__':
    main()

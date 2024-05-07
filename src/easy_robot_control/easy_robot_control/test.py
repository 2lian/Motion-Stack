"""
Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.time import Duration

class MoverNode(Node):
    def __init__(self):
        super().__init__("node")  # type: ignore
        self.get_logger().warn("start")
        self.startup_timer = self.create_timer(
            timer_period_sec=1/100,
            callback=self.tmr_callback,
        )

    def tmr_callback(self) -> None:
        return
        # self.get_logger().warn("tik")
        # self.startup_timer.cancel()
        wait = self.create_rate(1000)
        wait.sleep()
        wait._timer.destroy()
        wait.destroy()
        # self.get_clock().sleep_for(Duration(seconds=1/1000))
        # time.sleep(1/1000)

def main(args=None):
    rclpy.init()
    node = MoverNode()
    executor = rclpy.executors.MultiThreadedExecutor()  # ignore
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import time
import numpy as np

import ros2_numpy as rnp
import rclpy
from rclpy.node import Node

import python_package_include.maps as maps

from sensor_msgs.msg import PointCloud2

# x_map2 = np.arange(-500, 501, 100)
# y_map2 = np.arange(-500, 501, 100)
# z_map2 = -50
# X_map2, Y_map2, Z_map2 = np.meshgrid(x_map2, y_map2, z_map2)
#
# step_map = np.concatenate([X_map2.flatten().reshape((len(X_map2.flatten()), 1)),
#                            Y_map2.flatten().reshape((len(Y_map2.flatten()), 1)),
#                            Z_map2.flatten().reshape((len(Z_map2.flatten()), 1))], axis=1).astype('float32')
# step_map = np.concatenate([step_map, step_map + np.array([1000, 0, 200])])


class MyNode(Node):

    def __init__(self):
        super().__init__(f'pcl_static_pub')  # type: ignore

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.map_pub = self.create_publisher(PointCloud2, "map_pcl", 10)
        self.reach_pub = self.create_publisher(PointCloud2, "robot_reach", 10)
        self.leg0_reach_old_pub = self.create_publisher(
            PointCloud2, "leg0_reach_old", 10)
        self.leg0_reach_pub = self.create_publisher(
            PointCloud2, "leg0_reach", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Timers V
        #   \  /   #
        #    \/    #
        # self.tip_pub_timer = self.create_timer(1, self.step_pub)
        self.tip_pub_timer = self.create_timer(1, self.map_pub_func)
        self.reach_pud_timer = self.create_timer(1, self.robot_reach_pub)
        self.reach_old_pud_timer = self.create_timer(10, self.leg0_old_pub)
        self.reach_pud_timer = self.create_timer(10, self.leg0_pub)
        #    /\    #
        #   /  \   #
        # ^ Timers ^

    def npArr3coll_to_PclMsg(self, arr: np.ndarray):
        data = np.empty(arr.shape[0], dtype=[
            ('x', arr.dtype),
            ('y', arr.dtype),
            ('z', arr.dtype),
        ])
        data['x'] = arr[:, 0]/1000
        data['y'] = arr[:, 1]/1000
        data['z'] = arr[:, 2]/1000

        msg = rnp.msgify(PointCloud2, data,
                         stamp=self.get_clock().now().to_msg(),
                         frame_id="world")
        return msg

    def npArr3coll_Intensity_to_PclMsg(self, arr: np.ndarray, intensity: np.ndarray):
        data = np.empty(arr.shape[0], dtype=[
            ('x', arr.dtype),
            ('y', arr.dtype),
            ('z', arr.dtype),
            ("intensity", intensity.dtype)
        ])
        data['x'] = arr[:, 0]/1000
        data['y'] = arr[:, 1]/1000
        data['z'] = arr[:, 2]/1000
        data['intensity'] = intensity

        msg = rnp.msgify(PointCloud2, data,
                         stamp=self.get_clock().now().to_msg(),
                         frame_id="world")
        return msg

    def map_pub_func(self):
        with open("/home/elian/moonbot_software/src/pcl_reader/pcl_reader/python_package_include/map.npy", "rb") as file:
            arr = np.load(file).astype(np.float32) + \
                np.array([0, 0, -25], dtype=np.float32)
            # arr =np.load("python_package_include/robot_reach.npy")
        self.map_pub.publish(self.npArr3coll_to_PclMsg(arr))

    def leg0_old_pub(self):
        with open("/home/elian/moonbot_software/src/pcl_reader/pcl_reader/python_package_include/leg0_reach_old.npy", "rb") as file:
            arr = np.load(file).astype(np.float32) * \
                np.array([-1, 1, 1], dtype=np.float32)
            # arr =np.load("python_package_include/robot_reach.npy")
        msg = self.npArr3coll_to_PclMsg(arr)
        msg.header.frame_id = "base_link"
        self.leg0_reach_old_pub.publish(msg)

    def leg0_pub(self):
        with open("/home/elian/moonbot_software/src/pcl_reader/pcl_reader/python_package_include/leg0_reach.npy", "rb") as file:
            arr = np.load(file).astype(np.float32) + \
                np.array([0, 0, 0], dtype=np.float32)
            # arr =np.load("python_package_include/robot_reach.npy")
        msg = self.npArr3coll_to_PclMsg(arr)
        msg.header.frame_id = "base_link"
        self.leg0_reach_pub.publish(msg)

    # def step_pub(self):
    #     self.map_pub.publish(self.npArr3coll_to_PclMsg(
    #         step_map.astype(np.float32)))
    #
    def robot_reach_pub(self):
        with open("/home/elian/moonbot_software/src/pcl_reader/pcl_reader/python_package_include/robot_reach.npy", "rb") as file:
            arr = np.load(file).astype(np.float32)
            # arr =np.load("python_package_include/robot_reach.npy")
        with open("/home/elian/moonbot_software/src/pcl_reader/pcl_reader/python_package_include/robot_reach_intens.npy", "rb") as file:
            intensity = np.load(file)  # .astype(int)
            # arr =np.load("python_package_include/robot_reach.npy")
        self.reach_pub.publish(
            self.npArr3coll_Intensity_to_PclMsg(arr, intensity))


def main():
    rclpy.init()
    joint_state_publisher = MyNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(joint_state_publisher)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        joint_state_publisher.get_logger().debug(
            'KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
    joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

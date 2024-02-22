import time
import numpy as np

import ros2_numpy as rnp
import rclpy
from rclpy.node import Node

import python_package_include.maps as maps

from sensor_msgs.msg import PointCloud2

x_map2 = np.arange(-500, 501, 200)
y_map2 = np.arange(-500, 501, 200)
z_map2 = 0
X_map2, Y_map2, Z_map2 = np.meshgrid(x_map2, y_map2, z_map2)

step_map = np.concatenate([X_map2.flatten().reshape((len(X_map2.flatten()), 1)),
                           Y_map2.flatten().reshape((len(Y_map2.flatten()), 1)),
                           Z_map2.flatten().reshape((len(Z_map2.flatten()), 1))], axis=1).astype('float32')
step_map = np.concatenate([step_map, step_map + np.array([500, 0, 100])])

class MyNode(Node):

    def __init__(self):
        super().__init__(f'pcl_static_pub')  # type: ignore

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.map_pub = self.create_publisher(PointCloud2, "map_pcl", 10)
        self.reach_pub = self.create_publisher(PointCloud2, "robot_reach", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Timers V
        #   \  /   #
        #    \/    #
        self.tip_pub_timer = self.create_timer(1, self.step_pub)
        # self.reach_pud_timer = self.create_timer(1, self.robot_reach_pub)
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

    
    def example(self):

        data = np.zeros(100, dtype=[
          ('x', np.float32),
          ('y', np.float32),
          ('z', np.float32),
          # ('vectors', np.float32, (3,))
        ])
        data['x'] = np.arange(100)
        # data['y'] = data['x']*2
        # data['vectors'] = np.arange(100)[:,np.newaxis]

        msg = rnp.msgify(PointCloud2, data, 
                         stamp=self.get_clock().now().to_msg(),
                         frame_id="world")
        self.map_pub.publish(msg)

    def minimap_pub(self):
        with open("/home/elian/moonbot_software/src/pcl_reader/pcl_reader/python_package_include/map.npy", "rb") as file:
            arr =np.load(file).astype(np.float32)
            # arr =np.load("python_package_include/robot_reach.npy")
        self.map_pub.publish(self.npArr3coll_to_PclMsg(arr))

    def step_pub(self):
        self.map_pub.publish(self.npArr3coll_to_PclMsg(step_map))

    def robot_reach_pub(self):
        with open("/home/elian/moonbot_software/src/pcl_reader/pcl_reader/python_package_include/robot_reach.npy", "rb") as file:
            arr =np.load(file).astype(np.float32)
            # arr =np.load("python_package_include/robot_reach.npy")
        with open("/home/elian/moonbot_software/src/pcl_reader/pcl_reader/python_package_include/robot_reach_intens.npy", "rb") as file:
            intensity =np.load(file)# .astype(int)
            # arr =np.load("python_package_include/robot_reach.npy")
        self.reach_pub.publish(self.npArr3coll_Intensity_to_PclMsg(arr, intensity))


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

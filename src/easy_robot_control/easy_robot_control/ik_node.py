import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
import python_package_include.inverse_kinematics as ik


class IKNode(Node):

    def __init__(self):
        super().__init__(f'ik_node')  # type: ignore

        bypass_alive_check = False

        self.necessary_clients = [self.create_client(
            Empty, f'rviz_interface_alive'), self.create_client(Empty, f'remapper_alive')]
        while not any([client.wait_for_service(timeout_sec=2) for client in self.necessary_clients]):
            self.get_logger().warning(
                f'''Waiting for lower level, check that the [rviz_interface_alive or dynamixel_interface_alive] service is running''')
            if bypass_alive_check:
                break

        self.get_logger().warning(f'''Lower level connected :)''')

        self.declare_parameter('leg_number', 0)
        self.leg_num = self.get_parameter(
            'leg_number').get_parameter_value().integer_value

        self.declare_parameter('bodyToCoxa', float())
        self.declare_parameter('coxaLength', float())
        self.declare_parameter('femurLength', float())
        self.declare_parameter('tibiaLength', float())

        self.declare_parameter('coxaMax', float())
        self.declare_parameter('coxaMin', float())
        self.declare_parameter('femurMax', float())
        self.declare_parameter('femurMin', float())
        self.declare_parameter('tibiaMax', float())
        self.declare_parameter('tibiaMin', float())

        bodyToCoxa = self.get_parameter(
            'bodyToCoxa').get_parameter_value().double_value
        coxaLength = self.get_parameter(
            'coxaLength').get_parameter_value().double_value
        femurLength = self.get_parameter(
            'femurLength').get_parameter_value().double_value
        tibiaLength = self.get_parameter(
            'tibiaLength').get_parameter_value().double_value

        coxaMax = self.get_parameter(
            'coxaMax').get_parameter_value().double_value
        coxaMin = self.get_parameter(
            'coxaMin').get_parameter_value().double_value
        femurMax = self.get_parameter(
            'femurMax').get_parameter_value().double_value
        femurMin = self.get_parameter(
            'femurMin').get_parameter_value().double_value
        tibiaMax = self.get_parameter(
            'tibiaMax').get_parameter_value().double_value
        tibiaMin = self.get_parameter(
            'tibiaMin').get_parameter_value().double_value

        self.leg_param = ik.LegParameters(
            body_to_coxa=bodyToCoxa,
            coxa_length=coxaLength,
            femur_length=femurLength,
            tibia_length=tibiaLength,
            coxaMax_degree=np.rad2deg(coxaMax),
            coxaMin_degree=np.rad2deg(coxaMin),
            femurMax_degree=np.rad2deg(femurMax),
            femurMin_degree=np.rad2deg(femurMin),
            tibiaMax_degree=np.rad2deg(tibiaMax),
            tibiaMin_degree=np.rad2deg(tibiaMin),
        )

        self.joints_angle_arr = np.zeros(3, dtype=float)

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.joint_pub_list = []
        for joint in range(3):
            pub = self.create_publisher(
                Float64, f'set_joint_{self.leg_num}_{joint}_real', 10)
            self.joint_pub_list.append(pub)

        self.pub_tip = self.create_publisher(
            Vector3, f'tip_pos_{self.leg_num}', 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Subscribers V
        #   \  /   #
        #    \/    #
        self.sub_rel_target = self.create_subscription(Vector3, f'set_ik_target_{self.leg_num}',
                                                       self.set_ik_cbk,
                                                       10
                                                       )
        self.sub_angle_0 = self.create_subscription(Float64, f'angle_{self.leg_num}_0',
                                                    self.angle_0_cbk,
                                                    10
                                                    )
        self.sub_angle_1 = self.create_subscription(Float64, f'angle_{self.leg_num}_1',
                                                    self.angle_1_cbk,
                                                    10
                                                    )
        self.sub_angle_2 = self.create_subscription(Float64, f'angle_{self.leg_num}_2',
                                                    self.angle_2_cbk,
                                                    10
                                                    )
        #    /\    #
        #   /  \   #
        # ^ Subscribers ^

        # V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(
            Empty, f'ik_{self.leg_num}_alive', lambda req, res: res)
        #    /\    #
        #   /  \   #
        # ^ Service ^

        # V Timers V
        #   \  /   #
        #    \/    #
        self.tip_pub_timer = self.create_timer(0.02, self.publish_tip_pos)
        self.tip_pub_timer.cancel()  # this timer executes 0.02 after every new angle received
        #    /\    #
        #   /  \   #
        # ^ Timers ^

    def set_ik_cbk(self, msg):
        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        angles = ik.leg_ik(leg_number=self.leg_num,
                           target=target,
                           previous_angles=self.joints_angle_arr,
                           leg_param=self.leg_param)
        for joint in range(3):
            msg = Float64()
            msg.data = angles[joint]
            self.joint_pub_list[joint].publish(msg)

    def angle_0_cbk(self, msg):
        self.joints_angle_arr[0] = msg.data
        if self.tip_pub_timer.is_canceled():
            self.tip_pub_timer.reset()

    def angle_1_cbk(self, msg):
        self.joints_angle_arr[1] = msg.data
        if self.tip_pub_timer.is_canceled():
            self.tip_pub_timer.reset()

    def angle_2_cbk(self, msg):
        self.joints_angle_arr[2] = msg.data
        if self.tip_pub_timer.is_canceled():
            self.tip_pub_timer.reset()

    def publish_tip_pos(self):
        msg = Vector3()
        tip_pos = ik.forward_kine_body_zero(
            self.joints_angle_arr, self.leg_num, self.leg_param)[-1, :]
        msg.x = tip_pos[0]
        msg.y = tip_pos[1]
        msg.z = tip_pos[2]
        self.pub_tip.publish(msg)
        self.tip_pub_timer.cancel()


def main():
    rclpy.init()
    joint_state_publisher = IKNode()
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

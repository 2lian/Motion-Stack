import time

import numpy as np
import rclpy
from rclpy.node import Node, ReentrantCallbackGroup
import tf2_ros

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped, Transform


class CallbackHolder:
    def __init__(self, leg, joint, parent_node, joint_state):
        self.leg = leg
        self.joint_state = joint_state
        self.joint = joint
        self.parent_node = parent_node
        self.parent_node.create_subscription(
            Float64,
            f'set_joint_{leg}_{joint}_real',
            self.set_joint_cbk,
            10)
        self.pub = self.parent_node.create_publisher(
            Float64,
            f'angle_{self.leg}_{self.joint}',
            10)

    def set_joint_cbk(self, msg):
        new_msg = Float64()
        if self.joint == 1:
            angle = msg.data + np.pi/2
        else:
            angle = msg.data
        new_msg.data = msg.data
        self.joint_state.position[self.leg * 3 + self.joint] = angle
        self.pub.publish(new_msg)
        if self.parent_node.tmr.is_canceled():
            self.parent_node.tmr.reset()
        return


class RVizInterfaceNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__('joint_state_rviz')

        #    node_list = self.get_node_names()
        rviz_is_running = False

        if not rviz_is_running:
            node_info = self.get_node_names_and_namespaces()
            for node_name, node_namespace in node_info:
                if node_name == "rviz2":
                    rviz_is_running = True
            self.get_logger().warning(
                f'''Waiting for rviz, check that the [/rviz] node is running''')

        while not rviz_is_running:
            node_info = self.get_node_names_and_namespaces()
            for node_name, node_namespace in node_info:
                if node_name == "rviz2":
                    rviz_is_running = True
            self.get_logger().info(
                f'''Waiting for rviz, check that the [/rviz] node is running''')
            time.sleep(1)

        self.get_logger().warning(f'''Rviz connected :)''')
        
        self.declare_parameter('std_movement_time', 1.5)
        self.movement_time = self.get_parameter(
            'std_movement_time').get_parameter_value().double_value

        leg_num_remapping = [1, 3, 4, 2]

        self.joint_state = JointState()
        self.joint_state.name = [
            f'Leg{leg_num_remapping[0]}_Joint1', f'Leg{leg_num_remapping[0]}_Joint2', f'Leg{leg_num_remapping[0]}_Joint3',
            f'Leg{leg_num_remapping[1]}_Joint1', f'Leg{leg_num_remapping[1]}_Joint2', f'Leg{leg_num_remapping[1]}_Joint3',
            f'Leg{leg_num_remapping[2]}_Joint1', f'Leg{leg_num_remapping[2]}_Joint2', f'Leg{leg_num_remapping[2]}_Joint3',
            f'Leg{leg_num_remapping[3]}_Joint1', f'Leg{leg_num_remapping[3]}_Joint2', f'Leg{leg_num_remapping[3]}_Joint3',
        ]
        self.joint_state.position = [0.0] * (3 * 4)
        self.set_joint_subs = []
        self.loop_rate = 100  # Hz

        # V Subscriber V
        #   \  /   #
        #    \/    #
        cbk_holder_list = []
        for leg in range(4):
            for joint in range(3):
                holder = CallbackHolder(leg, joint, self, self.joint_state)
                cbk_holder_list.append(holder)
        self.body_pose_sub = self.create_subscription(
            Transform, "robot_body", self.robot_body_pose_cbk, 10)
        self.smooth_body_pose_sub = self.create_subscription(
            Transform, "smooth_body_rviz", self.smooth_body_trans, 10, callback_group=ReentrantCallbackGroup())
        #    /\    #
        #   /  \   #
        # ^ Subscriber ^

        # V Publisher V
        #   \  /   #
        #    \/    #
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10)
        # self.body_pose_pub = self.create_publisher(
        # TFMessage,
        # '/BODY', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        #    /\    #
        #   /  \   #
        # ^ Publisher ^

        self.tmr = self.create_timer(
            1 / self.loop_rate,
            self.publish_joint_state)

        # V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(
            Empty, 'rviz_interface_alive', lambda:  None)
        #    /\    #
        #   /  \   #
        # ^ Service ^

        # V Service V
        #   \  /   #
        #    \/    #
        self.body_refresh_timer = self.create_timer(0.1, self.body_refresh)
        #    /\    #
        #   /  \   #
        # ^ Service ^
        self.current_body_tra = np.array([0, 0, 0], dtype=float)
        self.current_body_rot = np.array([0, 0, 0, 1], dtype=float)
        # self.movement_time = 1.5 # is a ros param
        self.movement_update_rate = self.loop_rate

    def body_refresh(self):
        tra = self.current_body_tra
        rot = self.current_body_rot 
        msg = Transform()
        msg.translation.x, msg.translation.y, msg.translation.z = tuple(
            tra.tolist())
        msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w = tuple(
            rot.tolist())

        new_transform = TransformStamped()
        new_transform.header.stamp = self.get_clock().now().to_msg()
        new_transform.header.frame_id = 'world'
        new_transform.child_frame_id = 'base_link'
        new_transform.transform = msg
        self.tf_broadcaster.sendTransform(new_transform)
        return

    def robot_body_pose_cbk(self, msg):
        tra = np.array([msg.translation.x, msg.translation.y,
                       msg.translation.z], dtype=float)
        rot = np.array([msg.rotation.x, msg.rotation.y,
                       msg.rotation.z, msg.rotation.w], dtype=float)
        self.current_body_tra = tra
        self.current_body_rot = rot

        new_transform = TransformStamped()
        new_transform.header.stamp = self.get_clock().now().to_msg()
        new_transform.header.frame_id = 'world'
        new_transform.child_frame_id = 'base_link'
        new_transform.transform = msg
        self.tf_broadcaster.sendTransform(new_transform)

    def smooth_body_trans(self, request):
        tra = self.current_body_tra + np.array([request.translation.x, request.translation.y,
                        request.translation.z], dtype=float) / 1000
        rot = self.current_body_rot + np.array([request.rotation.x, request.rotation.y,
                        request.rotation.z, request.rotation.w], dtype=float) / 1000

        samples = int(self.movement_time * self.movement_update_rate)
        rate = self.create_rate(self.movement_update_rate)

        start_tra = self.current_body_tra
        start_rot = self.current_body_rot

        for x in np.linspace(0 + 1/samples, 1, num=samples):
            x = (1 - np.cos(x * np.pi)) / 2
            intermediate_tra = tra * x + start_tra * (1 - x)
            intermediate_rot = rot * x + start_rot * (1 - x)

            msg = Transform()
            msg.translation.x, msg.translation.y, msg.translation.z = tuple(
                intermediate_tra.tolist())
            msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w = tuple(
                intermediate_rot.tolist())
            self.robot_body_pose_cbk(msg)
            rate.sleep()
        return

    def publish_joint_state(self):
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state_pub.publish(self.joint_state)
        self.tmr.reset()


def main(args=None):
    rclpy.init()
    joint_state_publisher = RVizInterfaceNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joint_state_publisher)
    try:
        executor.spin()
    except KeyboardInterrupt:
        joint_state_publisher.get_logger().debug(
            'KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
    joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

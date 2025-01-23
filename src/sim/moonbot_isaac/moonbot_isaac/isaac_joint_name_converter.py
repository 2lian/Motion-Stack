import time
from xml.dom.minidom import Node as DomNode
from xml.dom.minidom import parseString

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class JointStateConverter(Node):
    """
    Prepare the robot description for Isaac Sim
    """

    def __init__(self):
        super().__init__("joint_state_converter")

        self.get_logger().info("Starting joint state converter...")
        self.callback_group = ReentrantCallbackGroup()

        # Find a service with the name including /robot_state_publisher/get_parameters
        # (Currently, the full robot description is published in the namespace of each leg)
        get_parameters_services = None
        while get_parameters_services is None:
            for service_name, service_types in self.get_service_names_and_types():
                if (
                    service_name.endswith("/robot_state_publisher/get_parameters")
                    and "rcl_interfaces/srv/GetParameters" in service_types
                ):
                    get_parameters_services = service_name
                    break

            if get_parameters_services is None:
                self.get_logger().info(
                    "Waiting for service /*/robot_state_publisher/get_parameters..."
                )
                time.sleep(1)

        self.param_client = self.create_client(
            GetParameters,
            get_parameters_services,
            callback_group=self.callback_group,
        )
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for service /robot_state_publisher/get_parameters..."
            )

        # Update the /robot_description and publish to Isaac
        # QoS profile that publishes the the last message for each new subscriber
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.robot_description_pub = self.create_publisher(
            String,
            "/robot_description_isaac",
            qos_profile,
            callback_group=self.callback_group,
        )

        # Convert the joint names from Isaac and republish to ROS
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states_isaac",
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", 10, callback_group=self.callback_group
        )

        # Convert the joint names from ROS and republish to Isaac
        self.joint_command_sub = self.create_subscription(
            JointState,
            "/joint_commands",
            self.joint_command_callback,
            10,
            callback_group=self.callback_group,
        )
        self.joint_command_pub = self.create_publisher(
            JointState, "/joint_commands_isaac", 10, callback_group=self.callback_group
        )

        # Publish the joint state errors in a separate topic
        self.joint_state_error_pub = self.create_publisher(
            JointState, "/joint_state_errors", 10, callback_group=self.callback_group
        )
        # save the last joint states and commands for comparision (using the ROS names)
        self.last_joint_states = {}
        self.last_joint_commands = {}
        self.error_pub_timer = self.create_timer(
            0.1, self.publish_joint_state_errors, callback_group=self.callback_group
        )

    def load_robot_description(self, executor):
        """
        Load the robot description from the /robot_state_publisher node params
        """
        request = GetParameters.Request()
        request.names = ["robot_description"]

        future = self.param_client.call_async(request)
        executor.spin_until_future_complete(future)

        if future.result() is not None:
            response = future.result()
            if response.values:
                robot_description = response.values[0].string_value
            else:
                raise ValueError("Parameter 'robot_description' not set or empty.")
        else:
            raise RuntimeError("Service call failed!")

        robot_description = self.process_robot_description(robot_description)
        self.robot_description_pub.publish(String(data=robot_description))

        # Add robot description as a node parameter
        self.declare_parameter("robot_description", robot_description)

    def process_robot_description(self, urdf):
        doc = parseString(urdf)

        # Change the robot name so all robots will have the same USD path in Isaac
        robot_element = doc.getElementsByTagName("robot")[0]
        robot_element.setAttribute("name", "robot")

        # Remove comments from the URDF description
        def remove_comments(node):
            if node.nodeType == DomNode.COMMENT_NODE:
                node.parentNode.removeChild(node)
            else:
                for child in node.childNodes:
                    remove_comments(child)

        remove_comments(doc)

        # Change the joint names to be compatible with Isaac Sim
        elements_with_name = doc.getElementsByTagName("*")
        for elem in elements_with_name:
            for attr_name in ["name", "link"]:
                if elem.hasAttribute(attr_name):
                    ros_name = elem.getAttribute(attr_name)
                    if ros_name == "":
                        continue
                    self.validate_name(ros_name)

        updated_urdf = doc.toxml()
        return updated_urdf

    def validate_name(self, name):
        """
        Log and error if the joint name is not compatible with Isaac Sim
        """
        # Check if the name doesn't start with a number
        if name[0].isdigit():
            self.get_logger().error(
                f"Joint name '{name}' is not compatible with Isaac Sim (starts with a number)"
            )

        # Check if the name doesn't contain any hyphens
        if "-" in name:
            self.get_logger().error(
                f"Joint name '{name}' is not compatible with Isaac Sim (contains a hyphen)"
            )

    def joint_state_callback(self, msg):
        """
        Receive the joint states from Isaac Sim and republish to ROS
        """
        self.joint_state_pub.publish(msg)
        # Save the joint states for comparison
        self.last_joint_states.update(
            {name: pos for name, pos in zip(msg.name, msg.position)}
        )

    def joint_command_callback(self, msg):
        """
        Receive the joint commands from ROS and republish to Isaac Sim
        """
        # Save the joint commands for comparison
        self.last_joint_commands.update(
            {name: pos for name, pos in zip(msg.name, msg.position)}
        )

        # Publish all the joints in each message
        msg.name = list(self.last_joint_commands.keys())
        msg.position = list(self.last_joint_commands.values())

        self.joint_command_pub.publish(msg)

    def publish_joint_state_errors(self):
        """
        Publish the joint state errors between Isaac and ROS
        """
        error_msg = JointState()
        error_msg.name = []
        error_msg.position = []
        for name, pos in self.last_joint_states.items():
            if name not in self.last_joint_commands:
                continue
            if pos != self.last_joint_commands[name]:
                error_msg.name.append(name)
                error_msg.position.append(pos - self.last_joint_commands[name])
        if error_msg.name:
            self.joint_state_error_pub.publish(error_msg)


def main(args=None):
    rclpy.init(args=args)

    joint_state_converter = JointStateConverter()
    executor = MultiThreadedExecutor()
    executor.add_node(joint_state_converter)

    joint_state_converter.load_robot_description(executor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        joint_state_converter.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class IsaacRobotDescriptionPublisher(Node):
    """
    Find a service with the name including /robot_state_publisher/get_parameters and
    republish the robot description as /robot_description_isaac
    (Currently, the full robot description is published in the namespace of each leg)
    """

    def __init__(self):
        super().__init__("isaac_robot_description_publisher")

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
                time.sleep(1)

        self.get_logger().info(
            f"Found robot description service: {get_parameters_services}"
        )

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

        self.robot_description_pub.publish(String(data=robot_description))

        # Add robot description as a node parameter
        self.declare_parameter("robot_description", robot_description)



def main(args=None):
    rclpy.init(args=args)

    joint_state_converter = IsaacRobotDescriptionPublisher()
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

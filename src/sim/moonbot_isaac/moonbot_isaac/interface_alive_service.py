import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class InterfaceAliveService(Node):
    def __init__(self):
        super().__init__("interface_alive_service")
        self.srv = self.create_service(Empty, "/rviz_interface_alive", self.callback)

    def callback(self, _request, response):
        return response


def main(args=None):
    rclpy.init(args=args)

    interface_alive_service = InterfaceAliveService()

    try:
        rclpy.spin(interface_alive_service)
    except KeyboardInterrupt:
        interface_alive_service.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

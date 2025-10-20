import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        # Create a client for the Spawn service
        self.client = self.create_client(Spawn, 'spawn')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')

        # Create request
        request = Spawn.Request()
        request.x = 5.0     # X position
        request.y = 5.0     # Y position
        request.theta = 0.0 # Orientation in radians
        request.name = "turtle2"  # Name of the new turtle

        # Call service asynchronously
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle named: {response.name}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin_once(node, timeout_sec=5)  # Spin briefly until response received
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

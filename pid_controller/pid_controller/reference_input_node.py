import rclpy
from rclpy.node import Node
from pid_controller_msgs.srv import SetReference

class ReferenceInputNode(Node):
    def __init__(self):
        super().__init__('reference_input_node')
        self.cli = self.create_client(SetReference, 'set_reference')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_reference service...')

        self.req = SetReference.Request()

    def send_request(self, value):
        self.req.reference = value
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = ReferenceInputNode()

    try:
        while True:
            try:
                user_input = float(input("Sett inn ny referanse (rad, -π til π):\n"))
                response = node.send_request(user_input)
                if response.success:
                    node.get_logger().info("Referanse satt!")
                else:
                    node.get_logger().warn("Referanse utenfor gyldig område.")
            except ValueError:
                print("Ugyldig tall, prøv igjen.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
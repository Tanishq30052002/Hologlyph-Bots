import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Empty  # Import the Empty service message

class FinishSubscriber(Node):

    def __init__(self):
        super().__init__('StopServiceNode_Grader')

        # Subscribers for each bot
        self.subscriber_bot1 = self.create_subscription(Bool, '/finish/bot1', self.bot1_callback, 1)
        self.subscriber_bot2 = self.create_subscription(Bool, '/finish/bot2', self.bot2_callback, 1)
        self.subscriber_bot3 = self.create_subscription(Bool, '/finish/bot3', self.bot3_callback, 1)

        self.bot1_finish = False
        self.bot2_finish = False
        self.bot3_finish = False

        # Create a client to call the stop service
        self.stop_service_client = self.create_client(Empty, '/Stop_Flag')

    def bot1_callback(self, msg):
        if msg.data:
            self.get_logger().info("Bot 1 has finished.")
            self.bot1_finish = True
            self.check_all_finished()

    def bot2_callback(self, msg):
        if msg.data:
            self.get_logger().info("Bot 2 has finished.")
            self.bot2_finish = True
            self.check_all_finished()

    def bot3_callback(self, msg):
        if msg.data:
            self.get_logger().info("Bot 3 has finished.")
            self.bot3_finish = True
            self.check_all_finished()

    def check_all_finished(self):
        if self.bot1_finish and self.bot2_finish and self.bot3_finish:
            self.start_service_server()

    def start_service_server(self):
        try:
            self.get_logger().info("Creating service")
            srv = self.create_service(Empty, 'Stop_Flag', self.stop_service_callback)
            self.get_logger().info("Created service")
        except Exception as e:
            self.get_logger().info(f"Error making service : {e}")

    def stop_service_callback(self, request, response):
        self.get_logger().info("Stop service request received.")
        # Implement your logic to stop the service here
        self.get_logger().info("Service stopped.")
        return response


def main(args=None):
    rclpy.init(args=args)

    finish_subscriber = FinishSubscriber()

    rclpy.spin(finish_subscriber)

if __name__ == '__main__':
    main()

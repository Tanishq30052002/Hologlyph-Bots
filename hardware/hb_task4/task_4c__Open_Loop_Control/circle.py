import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class BotCircle(Node):
    def __init__(self):
        super().__init__('circle')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel/bot3', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.i = 0
        self.counter = 130
        self.msg = Twist()

        self.left = 144.0
        self.right = 0.0
        self.rear = 0.0

    def stop(self):
        self.msg.linear.x = 90.0
        self.msg.linear.y = 90.0
        self.msg.linear.z = 90.0
        self.publisher_.publish(self.msg)

    def timer_callback(self):
        self.msg.linear.x = self.left
        self.msg.linear.y = self.right
        self.msg.linear.z = self.rear

        self.publisher_.publish(self.msg)
        self.get_logger().info(f'{self.msg.linear}')
        self.i += 1
        if self.i>self.counter:
            self.stop()
            # exit()

def main(args=None):
    rclpy.init(args=args)
    bot_publisher = BotCircle()
    rclpy.spin(bot_publisher)
    bot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

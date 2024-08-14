import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Square(Node):
    def __init__(self):
        super().__init__('square')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel/bot1', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.i = 1
        self.counter = 10
        self.msg = Twist()
        self.phase = 0
        self.up = [180.0, 27.0, 90.0]
        self.down = [0.0, 152.0, 90.0]
        self.left = [62.0, 62.0, 163.0]
        self.right = [125.0, 135.0, 25.0]

    def stop(self):
        self.msg.linear.x = 90.0
        self.msg.linear.y = 90.0
        self.msg.linear.z = 90.0
        self.publisher_.publish(self.msg)

    def timer_callback(self):
        if self.phase == 4:
            self.stop()
            # exit()
        if self.i % self.counter == 0:
            self.stop()
            self.phase += 1
        if self.phase == 0:
            self.msg.linear.x = self.up[0]
            self.msg.linear.y = self.up[1]
            self.msg.linear.z = self.up[2]
        elif self.phase == 1:
            self.msg.linear.x = self.right[0]
            self.msg.linear.y = self.right[1]
            self.msg.linear.z = self.right[2]
        elif self.phase == 2:
            self.msg.linear.x = self.down[0]
            self.msg.linear.y = self.down[1]
            self.msg.linear.z = self.down[2]
        elif self.phase == 3:
            self.msg.linear.x = self.left[0]
            self.msg.linear.y = self.left[1]
            self.msg.linear.z = self.left[2]


        self.publisher_.publish(self.msg)

        self.get_logger().info(f'{self.msg.linear}')

        self.i += 1 

def main(args=None):
    rclpy.init(args=args)
    square = Square()
    rclpy.spin(square)
    square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
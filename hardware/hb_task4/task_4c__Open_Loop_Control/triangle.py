import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Triangle(Node):
    def __init__(self):
        super().__init__('triangle')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel/bot2', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.i = 1
        self.counter = 10
        self.msg = Twist()
        self.phase = 0
        self.slopeUp =  [180.0, 64.0, 58.0]
        self.slopeDown = [80.0, 165.0, 48.0]
        self.flat = [64.0, 63.0, 180.0]

    def stop(self):
        self.msg.linear.x = 90.0
        self.msg.linear.y = 90.0
        self.msg.linear.z = 90.0
        self.publisher_.publish(self.msg)

    
    def timer_callback(self):
        if self.phase == 3:
            # self.i = 1
            # self.phase = 0
            self.stop()
            # exit()

        if self.i % self.counter == 0:
            self.stop()
            self.phase += 1
        if self.phase == 0:
            self.msg.linear.x = self.slopeUp[0]
            self.msg.linear.y = self.slopeUp[1]
            self.msg.linear.z = self.slopeUp[2]
        elif self.phase == 1:
            self.msg.linear.x = self.slopeDown[0]
            self.msg.linear.y = self.slopeDown[1]
            self.msg.linear.z = self.slopeDown[2]
        elif self.phase == 2:
            self.msg.linear.x = self.flat[0]
            self.msg.linear.y = self.flat[1]
            self.msg.linear.z = self.flat[2]

        self.publisher_.publish(self.msg)

        self.get_logger().info(f'{self.msg.linear}')

        self.i += 1 

def main(args=None):
    rclpy.init(args=args)
    triangle = Triangle()
    rclpy.spin(triangle)
    triangle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
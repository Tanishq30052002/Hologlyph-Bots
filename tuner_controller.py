import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2

class BotPublisher(Node):
    def __init__(self, ID):
        super().__init__('bot_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel/bot'+str(ID), 10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)
        self.i = 0
        self.msg = Twist()

        self.left = 90.0
        self.right = 90.0
        self.rear = 90.0

        # Create a window with a trackbar
        cv2.namedWindow('Trackbar Window')
        cv2.createTrackbar('Left', 'Trackbar Window', 90, 180, self.on_trackbar_changeleft)
        cv2.createTrackbar('Rigth', 'Trackbar Window', 90, 180, self.on_trackbar_changeright)
        cv2.createTrackbar('Rear', 'Trackbar Window', 90, 180, self.on_trackbar_changerear)

    def timer_callback(self):
        self.msg.linear.x = self.left
        self.msg.linear.y = self.right
        self.msg.linear.z = self.rear


        self.publisher_.publish(self.msg)
        self.get_logger().info(f"{self.msg.linear}")
        self.i += 1

        # Add the following line to update the GUI
        cv2.waitKey(1)

    def on_trackbar_changeleft(self, value):
        # Update the variable when the trackbar is moved
        value=value-90
        self.left = float(value)
    

    def on_trackbar_changeright(self, value):
        # Update the variable when the trackbar is moved
        value=value-90
        self.right = float(value)

    
    def on_trackbar_changerear(self, value):
        # Update the variable when the trackbar is moved
        value=value-90
        self.rear = float(value)

def main(args=None):
    rclpy.init(args=args)
    bot_publisher = BotPublisher(3)
    rclpy.spin(bot_publisher)
    bot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

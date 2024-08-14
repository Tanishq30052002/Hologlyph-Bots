########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1B ###########################################
# Team ID:2977
# Team Leader Name:Pranay Pandey
# Team Members Name:Aman Pandey, Pranay Pandey, Tanishq Chaudhary
# College:IIT (ISM) Dhanbad
########################################################################################################################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

class HologlyphBotShapeController(Node):
    def __init__(self):
        super().__init__('Hologlyph_Bot_Shape_Controller_Node')

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.twist = Twist()
        
        self.rate = self.create_rate(100)
        
        self.hb_x, self.hb_y, self.hb_theta = 0, 0, 0
        self.Kp = .5

        self.next_goal_client = self.create_client(NextGoal, 'next_goal')      

        while not self.next_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.next_goal_request = NextGoal.Request() 
        
        self.index = 0 # index of list containing goal points
            
    def odom_callback(self, msg):
        self.hb_x = msg.pose.pose.position.x
        self.hb_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.hb_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def send_request(self, index):
        self.next_goal_request.request_goal = index
        self.next_goal_response = self.next_goal_client.call_async(self.next_goal_request)

def main(args=None):
    rclpy.init(args=args)
    
    hologlyph_bot = HologlyphBotShapeController()
    hologlyph_bot.send_request(hologlyph_bot.index)
    
    while rclpy.ok():
        if hologlyph_bot.next_goal_response.done():
            try:
                response = hologlyph_bot.next_goal_response.result()

            except Exception as e:
                hologlyph_bot.get_logger().info('Service call failed %r' % (e,))

            else:
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                hologlyph_bot.flag = response.end_of_list

                delta_x = x_goal - hologlyph_bot.hb_x
                delta_y = y_goal - hologlyph_bot.hb_y
                delta_theta = theta_goal - hologlyph_bot.hb_theta

                delta_x_hb_frame = round(delta_x * math.cos(hologlyph_bot.hb_theta) + delta_y * math.sin(hologlyph_bot.hb_theta), 2)
                delta_y_hb_frame = round(-delta_x * math.sin(hologlyph_bot.hb_theta) + delta_y * math.cos(hologlyph_bot.hb_theta), 2)
                delta_theta_hb_frame = round(delta_theta, 2)

                error=0.3
                if not (abs(delta_x_hb_frame) < error and abs(delta_y_hb_frame) < error and abs(delta_theta_hb_frame) < error):
                    hologlyph_bot.twist.linear.x = hologlyph_bot.Kp * delta_x_hb_frame
                    hologlyph_bot.twist.linear.y = hologlyph_bot.Kp * delta_y_hb_frame
                    hologlyph_bot.twist.angular.z = hologlyph_bot.Kp * delta_theta_hb_frame
                    hologlyph_bot.cmd_publisher.publish(hologlyph_bot.twist)
                
                else:
                    hologlyph_bot.index += 1
                    if hologlyph_bot.flag == 1 :
                        hologlyph_bot.index = 0
                    hologlyph_bot.send_request(hologlyph_bot.index)

        rclpy.spin_once(hologlyph_bot)
    
    hologlyph_bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit
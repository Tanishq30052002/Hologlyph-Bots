import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Pose2D
from my_robot_interfaces.msg import Goal

import numpy as np
import math

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv

class HB_Conroller(Node):
    def __init__(self, bot_id):
        super().__init__(node_name='hb'+str(bot_id)+'_controller')
        self.id = bot_id
        
        self.pose_subscriber = self.create_subscription(Pose2D, '/detected_aruco_'+str(bot_id), self.pose_callback, 10)
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        self.visited = []
        self.index = 0

        self.goal_subscriber = self.create_subscription(Goal, '/hb_bot_'+str(bot_id)+'/goal', self.goal_callback, 10)
        self.goal = []
        
        self.left_wheel_force_publisher = self.create_publisher(Wrench, '/hb_bot_'+str(bot_id)+'/left_wheel_force', 10)
        self.right_wheel_force_publisher = self.create_publisher(Wrench, '/hb_bot_'+str(bot_id)+'/right_wheel_force', 10)
        self.rear_wheel_force_publisher = self.create_publisher(Wrench, '/hb_bot_'+str(bot_id)+'/rear_wheel_force', 10)
        self.left_wheel_force = Wrench()
        self.right_wheel_force = Wrench()
        self.rear_wheel_force = Wrench()

        self.image_subsciber = self.create_subscription(Image, '/detected_aruco_image', self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/hb_tracker_'+ str(bot_id), 10)
        self.cv_image = np.zeros((500, 500, 3), dtype=np.uint8)
        self.bridge = CvBridge()

    def pose_callback(self, msg):
        self.pose['x'] = int(msg.x)
        self.pose['y'] = int(msg.y)
        theta = msg.theta
        if theta > math.pi:
            theta = theta - 2*math.pi
        self.pose['theta'] = round(theta, 2)
    
    def goal_callback(self, msg):
        self.goal = msg
    
    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def inverse_kinematics(self, vx, vy, w):
        f_left = w/3 - vx/2 - vy*math.cos(math.pi/6)
        f_right = w/3 - vx/2 + vy*math.cos(math.pi/6)
        f_rear = w/3 + 0.7*vx

        return float(f_left), float(f_right), float(f_rear)

    def Tracker(self):


        for i in range(len(self.goal.x)):
            x = int(self.goal.x[i])
            y = int(self.goal.y[i])
            cv.circle(self.cv_image, (x, y),2, (0, 0, 255), -1)

        bot_x = int(self.pose['x'])
        bot_y = int(self.pose['y'])

        cv.circle(self.cv_image, (bot_x, bot_y), 5, (0, 255, 0), -1)

        self.visited.append((bot_x, bot_y))
        for point in self.visited:
            cv.circle(self.cv_image, point, 2, (255, 0, 0), -1)

        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding='passthrough'))
    
    def Controller(self):
        x_goal = self.goal.x[self.index]
        y_goal = self.goal.y[self.index]
        theta_goal = self.goal.theta

        cv.circle(self.cv_image, (int(x_goal), int(y_goal)), 10, (0, 0, 255), -1)

        x_bot = self.pose['x']
        y_bot = self.pose['y']
        theta_bot = self.pose['theta']

        del_x = x_goal - x_bot
        del_y = y_goal - y_bot
        
        del_theta = (theta_goal - theta_bot)%2*math.pi 
        if del_theta > math.pi:
            del_theta = del_theta - 2*math.pi

        # self.get_logger().info("Goal: {}, {}, {}".format(x_goal, y_goal, theta_goal))
        # self.get_logger().info("Bot: {}, {}, {}".format(x_bot, y_bot, theta_bot))
        
        # self.get_logger().info("Delta: {}, {}, {}".format(del_x, del_y, del_theta))

        delta_x = -del_y*math.sin(theta_bot) + del_x*math.cos(theta_bot)
        delta_y = del_x*math.sin(theta_bot) + del_y*math.cos(theta_bot)
        delta_theta = del_theta
        # self.get_logger().info("Delta_HB: {}, {}, {}".format(delta_x, delta_y, delta_theta))
        # self.get_logger().info("#"*5)
        # dist_error, theta_error = 10, 0.15
        dist_error, theta_error = 2, 0.2

        vx, vy, w = 0, 0, 0
        # self.get_logger().info(math.sqrt(delta_x**2 + delta_y**2))

        def bound(V, limit):
            if abs(V) > limit:
                V = limit*np.sign(V)
            return V
        # 
        if not (math.sqrt(delta_x**2 + delta_y**2) < dist_error and abs(delta_theta) < theta_error):
            kp_x, kp_y, kp_theta = 10, 10, 70
            vx = bound(V=kp_x*delta_x, limit=40)
            vy = bound(V=-kp_y*delta_y, limit=40)
            w = round(kp_theta*delta_theta, 2)
            # vx, vy, w = 0, 0, 0

            # print("Velocity: ", vx, vy, w)

        else:
            if self.index < len(self.goal.x)-1:
                self.index += 1
                # self.get_logger().info("Goal Reached")
            else:
                self.get_logger().info("All goals reached")

        f_left, f_right, f_rear = self.inverse_kinematics(vx, vy, w)
        # print("Force: ", f_left, f_right, f_rear)

        self.left_wheel_force.force.y = f_left
        self.right_wheel_force.force.y = f_right
        self.rear_wheel_force.force.y = f_rear

        self.left_wheel_force_publisher.publish(self.left_wheel_force)
        self.right_wheel_force_publisher.publish(self.right_wheel_force)
        self.rear_wheel_force_publisher.publish(self.rear_wheel_force)

        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding='passthrough'))

def main(args=None):
    rclpy.init(args=args)

    hb_list = []
    for bot_id in range(3):
        hb_list.append(HB_Conroller(bot_id+1))
    
    # Spin until we get the goal for all the bots
    while not all([bot.goal for bot in hb_list]):
        for bot in hb_list:
            rclpy.spin_once(bot)

    # Run custom function for each node continuously
    for bot in hb_list:
        bot.create_timer(0.1, bot.Tracker)
        bot.create_timer(0.1, bot.Controller)

    # Create a MultiThreadedExecutor
    executor = rclpy.executors.MultiThreadedExecutor()

    # Add your nodes to the executor
    for bot in hb_list:
        executor.add_node(bot)

    # Spin the nodes in parallel
    executor.spin()

if __name__ == '__main__':
    try:
        main()

    except KeyboardInterrupt:
        pass
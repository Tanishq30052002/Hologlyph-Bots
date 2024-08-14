import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Pose2D
from my_robot_interfaces.srv import NextGoal

import numpy as np
import math

import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class HB_Conroller(Node):
    def __init__(self):
        super().__init__(node_name='hb_controller')

        self.pose_subscriber = self.create_subscription(Pose2D, '/detected_aruco', self.pose_callback, 10)
        self.hb_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        self.left_wheel_force_publisher = self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.right_wheel_force_publisher = self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.rear_wheel_force_publisher = self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)

        self.left_wheel_force = Wrench()
        self.right_wheel_force = Wrench()
        self.rear_wheel_force = Wrench()

        self.image_subsciber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/hb_tracker', 10)
        self.cv_image = np.zeros((500, 500, 3), dtype=np.uint8)
        self.bridge = CvBridge()

        self.next_goal_client = self.create_client(NextGoal, 'next_goal')

        while not self.next_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.next_goal_request = NextGoal.Request()

        self.index = 0

        self.PointsVisited = []
        self.BotVisited = []

    def pose_callback(self, msg):
        self.hb_pose['x'] = int(msg.x) - 250
        self.hb_pose['y'] = 250 - int(msg.y)
        theta = msg.theta
        if theta > math.pi:
            theta = theta - 2*math.pi
        self.hb_pose['theta'] = round(theta, 2)

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def send_request(self, index):
        self.next_goal_request.request_goal = index
        self.next_goal_response = self.next_goal_client.call_async(self.next_goal_request)

    def inverse_kinematics(self, vx, vy, w):
        f_left = w/3 - vx/2 - vy*math.cos(math.pi/6)
        f_right = w/3 - vx/2 + vy*math.cos(math.pi/6)
        f_rear = w/3 + 0.7*vx

        return float(f_left), float(f_right), float(f_rear)

def main(args=None):
    rclpy.init(args=args)

    hola_bot = HB_Conroller()
    hola_bot.send_request(hola_bot.index)

    while rclpy.ok():
        if hola_bot.next_goal_response.done():
            try:
                response = hola_bot.next_goal_response.result()

            except Exception as e:
                hola_bot.get_logger().info('Service call failed %r' % (e,))

            else:
                x_goal_image = int(response.x_goal)
                y_goal_image = int(response.y_goal)
                theta_goal = response.theta_goal
                hola_bot.flag = response.end_of_list
                
                image = hola_bot.cv_image
                cv.circle(image, (x_goal_image+250, 250-y_goal_image),2, (0, 0, 255), -1)

                hola_bot.PointsVisited.append((x_goal_image+250, 250-y_goal_image))
                for point in hola_bot.PointsVisited:
                    cv.circle(image, point, 1, (0, 255, 0), -1)

                # print("#"*30)
                # print("World Goal: ", x_goal_world, y_goal_world, theta_goal)
                # print("Image Goal: ", x_goal_image, y_goal_image, theta_goal)
                # print("HB Pose: ", hola_bot.hb_pose['x'], hola_bot.hb_pose['y'], hola_bot.hb_pose['theta'])

                hola_bot.BotVisited.append((hola_bot.hb_pose['x']+250, 250-hola_bot.hb_pose['y']))
                for point in hola_bot.BotVisited:
                    cv.circle(image, point, 1, (255, 0, 0), -1)

                del_x = x_goal_image - hola_bot.hb_pose['x']
                del_y = y_goal_image - hola_bot.hb_pose['y']
                del_theta = theta_goal - hola_bot.hb_pose['theta']

                delta_x = -del_y*math.sin(hola_bot.hb_pose['theta']) + del_x*math.cos(hola_bot.hb_pose['theta'])
                delta_y = del_x*math.sin(hola_bot.hb_pose['theta']) + del_y*math.cos(hola_bot.hb_pose['theta'])
                delta_theta = del_theta
                
                dist_error, theta_error = 1, 0.1

                vx, vy, w = 0, 0, 0
                
                def bound(V, limit):
                    if abs(V) > limit:
                        V = limit*np.sign(V)
                    return V
                
                if not (math.sqrt(delta_x**2 + delta_y**2) < dist_error and abs(delta_theta) < theta_error):
                    kp_x, kp_y, kp_theta = 13, 13, 75
                    vx = bound(V=kp_x*delta_x, limit=40)
                    vy = bound(V=kp_y*delta_y, limit=40)
                    w = round(kp_theta*delta_theta, 2)

                else:
                    hola_bot.index += 1
                    if hola_bot.flag == 1:
                        hola_bot.left_wheel_force.force.y = 0.0
                        hola_bot.right_wheel_force.force.y = 0.0
                        hola_bot.rear_wheel_force.force.y = 0.0
                        hola_bot.left_wheel_force_publisher.publish(hola_bot.left_wheel_force)
                        hola_bot.right_wheel_force_publisher.publish(hola_bot.right_wheel_force)
                        hola_bot.rear_wheel_force_publisher.publish(hola_bot.rear_wheel_force)
                        hola_bot.get_logger().info('Reached the goal')
                        exit()
                        hola_bot.index = 0
                    hola_bot.send_request(hola_bot.index)

                f_left, f_right, f_rear = hola_bot.inverse_kinematics(vx, vy, w)
                # print("Force: ", f_left, f_right, f_rear)

                hola_bot.left_wheel_force.force.y = f_left
                hola_bot.right_wheel_force.force.y = f_right
                hola_bot.rear_wheel_force.force.y = f_rear

                hola_bot.left_wheel_force_publisher.publish(hola_bot.left_wheel_force)
                hola_bot.right_wheel_force_publisher.publish(hola_bot.right_wheel_force)
                hola_bot.rear_wheel_force_publisher.publish(hola_bot.rear_wheel_force)

                hola_bot.image_publisher.publish(hola_bot.bridge.cv2_to_imgmsg(image, encoding='passthrough'))

        rclpy.spin_once(hola_bot)

    hola_bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()

    except KeyboardInterrupt:
        pass

from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, String

import numpy as np
import math


class HolonomicBotController(Node):
    def __init__(self, robot_id, initialize, goal_positions, goal_positions_theta, errors, kp={"x": 1, "y": 1, "w": 1}, kd={"x": 0, "y": 0, "w": 0}, ki={"x": 0, "y": 0, "w": 0}, deadbands={'left': {'+': 0, '-': 0}, 'right': {'+': 0, '-': 0}, 'rear': {'+': 0, '-': 0}}):
        super().__init__(node_name=f'hb_{robot_id}_controller')
        self.robot_id = robot_id
        self.goal_positions_all = goal_positions
        self.goal_position_theta = goal_positions_theta
        self.deadbands = deadbands
        self.kp = kp
        self.kd=kd
        self.ki=ki
        self.errors = errors

        self.pose_subscriber = self.create_subscription(Pose2D, f'/pen{robot_id}_pose', self.pose_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, f"/cmd_vel/bot{robot_id}", 1)
        self.pen_pub = self.create_publisher(Bool, f"/pen{robot_id}_down", 1)
        self.bot_status_pub = self.create_publisher(Bool, f"/finish/bot{robot_id}", 1)
        self.collision_status_sub = self.create_subscription(String, f"/collision_status/bot{robot_id}", self.collision_status_callback, 1)

        self.current_pose = Pose2D()
        self.command_twist = Twist()
        self.pen_status = Bool()
        self.bot_status = Bool()
        self.collision_status=String()

        self.goal_index = 0
        self.current_contour=0

        self.current_goal_positions=[]
        self.reached_goal = False

        self.reached_start=False

        self.integral_error = {"x": 0, "y": 0, "theta": 0}
        self.previous_error = {"x": 0, "y": 0, "theta": 0}

        self.initializing=initialize

    def collision_status_callback(self, msg):
        self.collision_status.data=msg.data

    def pose_callback(self, pose_msg):
        self.current_pose.x = pose_msg.x
        self.current_pose.y = pose_msg.y
        self.current_pose.theta = pose_msg.theta

    def bound(self, value1, value2, value3, thresh=60):
        maxi = max(abs(value1), max(abs(value2), abs(value3)))
        if (maxi >= thresh):
            value1 = float(value1/maxi*thresh)
            value2 = float(value2/maxi*thresh)
            value3 = float(value3/maxi*thresh)
        return value1, value2, value3

    def deadband(self, value, wheel):
        if (value > 0):
            value = value+self.deadbands[wheel]['+']
        elif (value < 0):
            value = value-self.deadbands[wheel]['-']
        return value

    def inverse_kinematics(self, linear_velocity_x, linear_velocity_y, angular_velocity):
        left_wheel_velocity = (self.deadband(
            angular_velocity / 3 - linear_velocity_x / 2 - linear_velocity_y * math.cos(math.pi / 6), 'left'))
        right_wheel_velocity = (self.deadband(
            angular_velocity / 3 - linear_velocity_x / 2 + linear_velocity_y * math.cos(math.pi / 6), 'right'))
        rear_wheel_velocity = (self.deadband(
            angular_velocity / 3 + 0.6 * linear_velocity_x, 'rear'))

        left_wheel_velocity, right_wheel_velocity, rear_wheel_velocity = self.bound(left_wheel_velocity, right_wheel_velocity, rear_wheel_velocity)
        return float(left_wheel_velocity), float(right_wheel_velocity), float(rear_wheel_velocity)

    def control_robot(self):
        goal_x, goal_y = self.goal_positions_all[self.current_contour][self.goal_index]
        goal_theta = self.goal_position_theta

        current_x, current_y, current_theta = self.current_pose.x, self.current_pose.y, self.current_pose.theta

        delta_x, delta_y, delta_theta = goal_x - \
            current_x, goal_y - current_y, goal_theta - current_theta

        delta_x_bot_frame = -delta_y * \
            math.sin(current_theta) + delta_x * math.cos(current_theta)
        delta_y_bot_frame = delta_x * \
            math.sin(current_theta) + delta_y * math.cos(current_theta)
        delta_theta_bot_frame = delta_theta

        linear_error_threshold = self.errors[0]
        angular_error_threshold = self.errors[1]

        linear_velocity_x, linear_velocity_y, angular_velocity = 0, 0, 0
        pen_down = True if self.goal_index != 0 else False

        kp_x, kp_y, kp_theta = self.kp["x"], self.kp["y"], self.kp["w"]

        if abs(delta_theta_bot_frame) > angular_error_threshold or math.sqrt((delta_x_bot_frame ** 2) + (delta_y_bot_frame ** 2)) > linear_error_threshold:

            self.integral_error["x"] += delta_x_bot_frame
            self.integral_error["y"] += delta_y_bot_frame
            self.integral_error["theta"] += delta_theta_bot_frame

            linear_velocity_x = kp_x * delta_x_bot_frame + self.ki["x"] * self.integral_error["x"] + self.kd["x"] * (delta_x_bot_frame - self.previous_error["x"])
            linear_velocity_y = -kp_y * delta_y_bot_frame - self.ki["y"] * self.integral_error["y"] - self.kd["y"] * (delta_y_bot_frame - self.previous_error["y"])
            angular_velocity = kp_theta * delta_theta_bot_frame + self.ki["w"] * self.integral_error["theta"] + self.kd["w"] * (delta_theta_bot_frame - self.previous_error["theta"])
            
            speed=1.0
            if self.collision_status.data=="unsafe":
                speed=0.5
            elif self.collision_status.data=="danger":
                speed=-0.5

            linear_velocity_x=speed*linear_velocity_x
            linear_velocity_y=speed*linear_velocity_y
            linear_velocity_theta=speed*linear_velocity_theta
            
            self.previous_error["x"]=delta_x_bot_frame
            self.previous_error["y"]=delta_y_bot_frame
            self.previous_error["theta"]=delta_theta_bot_frame

        else:
            if self.goal_index < np.size(self.goal_positions_all[self.current_contour], axis=0):
                self.goal_index += 1
                self.integral_error["x"] = 0
                self.integral_error["y"] = 0
                self.integral_error["theta"] = 0

            if self.goal_index == np.size(self.goal_positions_all[self.current_contour], axis=0):
                linear_velocity_x, linear_velocity_y, angular_velocity = 0, 0, 0
                self.reached_goal = True
                self.goal_index -= 1

        if self.reached_goal:
            self.get_logger().info(f"contour {self.current_contour}, is done")
            self.current_contour+=1
            self.goal_index=0
            self.reached_goal=False

            self.reached_start=True

            if(self.current_contour>=len(self.goal_positions_all)):
                self.get_logger().info(f"BOT: {self.robot_id}, Reached Destination")
                
                linear_velocity_x, linear_velocity_y, angular_velocity =0.0, 0.0, 0.0
                left_wheel, right_wheel, rear_wheel = self.inverse_kinematics(linear_velocity_x, linear_velocity_y, angular_velocity)
                self.command_twist.linear.x = left_wheel
                self.command_twist.linear.y = right_wheel
                self.command_twist.linear.z = rear_wheel
                self.cmd_pub.publish(self.command_twist)

                pen_down=False
                self.pen_status.data = pen_down
                self.pen_pub.publish(self.pen_status)

                if not self.initializing:
                    self.bot_status.data=True
    
                self.bot_status_pub.publish(self.bot_status)

                self.destroy_timer(self.timer)
                return

        left_wheel, right_wheel, rear_wheel = self.inverse_kinematics(linear_velocity_x, linear_velocity_y, angular_velocity)
        self.command_twist.linear.x = left_wheel
        self.command_twist.linear.y = right_wheel
        self.command_twist.linear.z = rear_wheel
        self.cmd_pub.publish(self.command_twist)

        self.pen_status.data = pen_down
        self.pen_pub.publish(self.pen_status)

        # else:
        # self.get_logger().info(f"goal_x {round(goal_x,2)}, goal_y {round(goal_y,2)}, goal_theta {round(goal_theta,2)}")
        # self.get_logger().info(f"current /usr/bin/python3 /home/pranay/eyrc_23/eyrc_hb/hb_task_6_ws/task_6a__FunctionFollower.py _x {round(current_x,2)}, current_y {round(current_y,2)}, current_theta {round(current_theta,2)}")
        # self.get_logger().info(f"delta_x_bot {round(delta_x_bot_frame,2)}, delta_y_bot {round(delta_y_bot_frame,2)}, delta_theta_bot {round(delta_theta_bot_frame,2)}")
        # self.get_logger().info(f"left_wheel {round(left_wheel,2)}, right_wheel {round(right_wheel,2)}, rear_wheel {round(rear_wheel,2)}")
        # self.get_logger().info("#" * 50)

        

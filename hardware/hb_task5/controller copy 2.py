from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool

import numpy as np
import math


class HolonomicBotController(Node):
    def __init__(self, robot_id, goal_positions, goal_positions_theta, deadbands={'left':{'+':0, '-':0}, 'right':{'+':0, '-':0}, 'rear':{'+':0, '-':0}}):
        super().__init__(node_name=f'hb_{robot_id}_controller')
        self.robot_id = robot_id
        self.goal_positions = goal_positions
        self.goal_position_theta = goal_positions_theta
        self.deadbands=deadbands

        self.pose_subscriber = self.create_subscription(Pose2D, f'/pen{robot_id}_pose', self.pose_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, f"/cmd_vel/bot{robot_id}", 1)
        self.pen_pub = self.create_publisher(Bool, f"/pen{robot_id}_down", 1)
        self.bot_status_pub = self.create_publisher(Bool, f"/finish/bot{robot_id}", 1)

        self.current_pose = Pose2D()
        self.command_twist = Twist()
        self.pen_status = Bool()
        self.bot_status = Bool()

        self.goal_index = 0
        self.reached_goal = False

        self.lookback = 2
        self.prevLeft = [0, 0]
        self.prevRight = [0, 0]
        self.prevRear = [0, 0]

        self.movingAverageIndividual = False
        self.movingAverage = False

    def pose_callback(self, pose_msg):
        self.current_pose.x = pose_msg.x
        self.current_pose.y = pose_msg.y
        self.current_pose.theta = pose_msg.theta

    def bound(self, value1, value2, value3):
        max_val = max(value1, max(value2, value3))
        if max_val<35:
            return value1, value2, value3
        value1 = float(value1/max_val)*35
        value2 = float(value2/max_val)*35
        value3 = float(value3/max_val)*35
        return value1, value2, value3

    def deadband(self, value, wheel):
        if(value>0):
            value=value+self.deadbands[wheel]['+']
        elif(value<0):
            value=value-self.deadbands[wheel]['-']
        return value

    def ld_inverse_kinematics(self, linear_velocity_x, linear_velocity_y, angular_velocity, d=0.073, r=0.018):
        fw = (d*angular_velocity+linear_velocity_x) / r
        rw = (d*angular_velocity-linear_velocity_x/2-math.sqrt(3)*linear_velocity_y/2)/r
        lw = (d*angular_velocity-linear_velocity_x/2+math.sqrt(3)*linear_velocity_y/2)/r

        fw = self.bound(fw)
        rw = self.bound(rw)
        lw = self.bound(lw)

        return float(lw), float(rw), float(fw)

    def inverse_kinematics(self, linear_velocity_x, linear_velocity_y, angular_velocity):
        left_wheel_velocity = self.deadband(angular_velocity / 3 - linear_velocity_x / 2 - linear_velocity_y * math.cos(math.pi / 6), 'left')
        right_wheel_velocity = self.deadband(angular_velocity / 3 - linear_velocity_x / 2 + linear_velocity_y * math.cos(math.pi / 6), 'right')
        rear_wheel_velocity = self.deadband(angular_velocity / 3 + linear_velocity_x, 'rear')

        left_wheel_velocity, right_wheel_velocity, rear_wheel_velocity = self.bound(
            left_wheel_velocity, 
            right_wheel_velocity, 
            rear_wheel_velocity)

        return float(left_wheel_velocity), float(right_wheel_velocity), float(rear_wheel_velocity)

    def control_robot(self):
        goal_x, goal_y = self.goal_positions[self.goal_index]
        goal_theta = self.goal_position_theta

        current_x, current_y, current_theta = self.current_pose.x, self.current_pose.y, self.current_pose.theta

        delta_x, delta_y, delta_theta = goal_x - current_x, goal_y - current_y, goal_theta - current_theta

        if self.movingAverage:
            if len(self.prevLeft)<=self.lookback:
                leftSum = float((self.prevLeft[-1]+ left_wheel))
                rightSum = float((self.prevRight[-1]+ right_wheel))
                rearSum = float((self.prevRear[-1]+ rear_wheel))

                self.prevLeft[-1] = left_wheel
                self.prevRight[-1] = right_wheel
                self.prevRear[-1] = rear_wheel

                self.prevLeft.append(leftSum)
                self.prevRight.append(rightSum)
                self.prevRear.append(rearSum)

            else:
                leftSum = float(self.prevLeft[-1]-self.prevLeft[0]+left_wheel)
                rightSum = float(self.prevRight[-1]-self.prevRight[0]+right_wheel)
                rearSum = float(self.prevRear[-1]-self.prevRear[0]+rear_wheel)

                self.prevLeft.pop(0)
                self.prevRight.pop(0)
                self.prevRear.pop(0)
                
                self.prevLeft[-1] = left_wheel
                self.prevRight[-1] = right_wheel
                self.prevRear[-1] = rear_wheel

                self.prevLeft.append(leftSum)
                self.prevRight.append(rightSum)
                self.prevRear.append(rearSum)
            
            sz = len( self.prevLeft)
            left_wheel, right_wheel, rear_wheel = leftSum/sz, rightSum/sz, rearSum/sz

        delta_x_bot_frame = -delta_y * math.sin(current_theta) + delta_x * math.cos(current_theta)
        delta_y_bot_frame = delta_x * math.sin(current_theta) + delta_y * math.cos(current_theta)
        delta_theta_bot_frame = delta_theta

        linear_error_threshold = 10
        angular_error_threshold = 0.5

        linear_velocity_x, linear_velocity_y, angular_velocity = 0, 0, 0
        pen_down = True if self.goal_index != 0 else False

        kp_x, kp_y, kp_theta = 0.8, 0.8, 80
        
        if abs(delta_theta_bot_frame) > angular_error_threshold or math.sqrt((delta_x_bot_frame ** 2) + (delta_y_bot_frame ** 2)) > linear_error_threshold:
            linear_velocity_x = kp_x * delta_x_bot_frame
            linear_velocity_y = -kp_y * delta_y_bot_frame
            angular_velocity = kp_theta * delta_theta_bot_frame
        else:
            linear_velocity_x, linear_velocity_y, angular_velocity = 0, 0, 0
            if self.goal_index < np.size(self.goal_positions, axis=0):
                self.goal_index += 1

            if self.goal_index == np.size(self.goal_positions, axis=0):
                pen_down = False  # Goal Reached
                self.reached_goal = True
                self.goal_index -= 1

        left_wheel, right_wheel, rear_wheel = self.inverse_kinematics(linear_velocity_x, linear_velocity_y, angular_velocity)
        
        if self.movingAverageIndividual:
            if len(self.prevLeft)<=self.lookback:
                leftSum = float((self.prevLeft[-1]+ left_wheel))
                rightSum = float((self.prevRight[-1]+ right_wheel))
                rearSum = float((self.prevRear[-1]+ rear_wheel))

                self.prevLeft[-1] = left_wheel
                self.prevRight[-1] = right_wheel
                self.prevRear[-1] = rear_wheel

                self.prevLeft.append(leftSum)
                self.prevRight.append(rightSum)
                self.prevRear.append(rearSum)

            else:
                leftSum = float(self.prevLeft[-1]-self.prevLeft[0]+left_wheel)
                rightSum = float(self.prevRight[-1]-self.prevRight[0]+right_wheel)
                rearSum = float(self.prevRear[-1]-self.prevRear[0]+rear_wheel)

                self.prevLeft.pop(0)
                self.prevRight.pop(0)
                self.prevRear.pop(0)
                
                self.prevLeft[-1] = left_wheel
                self.prevRight[-1] = right_wheel
                self.prevRear[-1] = rear_wheel

                self.prevLeft.append(leftSum)
                self.prevRight.append(rightSum)
                self.prevRear.append(rearSum)
            
            sz = len( self.prevLeft)
            left_wheel, right_wheel, rear_wheel = leftSum/sz, rightSum/sz, rearSum/sz
                
        self.command_twist.linear.x = left_wheel
        self.command_twist.linear.y = right_wheel
        self.command_twist.linear.z = rear_wheel
        self.cmd_pub.publish(self.command_twist)
        
        self.pen_status.data = pen_down
        self.pen_pub.publish(self.pen_status)

        
        if self.reached_goal:
            self.bot_status.data=True
            # self.get_logger().info("Reached Goal")
        else:    
            self.bot_status.data=False
            # self.get_logger().info(f"goal_x {round(goal_x,2)}, goal_y {round(goal_y,2)}, goal_theta {round(goal_theta,2)}")
            # self.get_logger().info(f"current_x {round(current_x,2)}, current_y {round(current_y,2)}, current_theta {round(current_theta,2)}")
            self.get_logger().info(f"delta_x_bot {round(delta_x_bot_frame,2)}, delta_y_bot {round(delta_y_bot_frame,2)}, delta_theta_bot {round(delta_theta_bot_frame,2)}")
            # self.get_logger().info(f"left_wheel {round(left_wheel,2)}, right_wheel {round(right_wheel,2)}, rear_wheel {round(rear_wheel,2)}")
            # self.get_logger().info("#" * 50)

        self.bot_status_pub.publish(self.bot_status)
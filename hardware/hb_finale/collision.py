import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool
import argparse

import numpy as np
import math

R1 = 20
R2 = 30
class Avoidance(Node):
    
    def __init__(self):
        super().__init__(node_name="collision_avoidance")
        self.sub1 = self.create_subscription(Twist, "/cmd_vel/bot1", self.setPose1, 1)
        self.sub2 = self.create_subscription(Twist, "/cmd_vel/bot2", self.setPose2, 1)
        self.sub3 = self.create_subscription(Twist, "/cmd_vel/bot3", self.setPose3, 1)
        self.bot1Pose = Pose2D()
        self.bot2Pose = Pose2D()
        self.bot3Pose = Pose2D()
        self.pub1 = self.create_publisher(Bool, "/collision_avoidance/bot1", 1)
        self.pub2 = self.create_publisher(Bool, "/collision_avoidance/bot2", 1)
        self.pub3 = self.create_publisher(Bool, "/collision_avoidance/bot3", 1)
        self.B1slow = False
        self.B2slow = False
        self.B3slow = False

        self.timer = self.create_timer(0.1, self.calc)
        
    def setPose1(self, pose_msg):
        self.bot1Pose.x, self.bot1Pose.y, self.bot1Pose.theta = pose_msg.x, pose_msg.y, pose_msg.theta

    def setPose2(self, pose_msg):
        self.bot2Pose.x, self.bot2Pose.y, self.bot2Pose.theta = pose_msg.x, pose_msg.y, pose_msg.theta

    def setPose3(self, pose_msg):
        self.bot3Pose.x, self.bot3Pose.y, self.bot3Pose.theta = pose_msg.x, pose_msg.y, pose_msg.theta

    def calc(self):
        if self.B1slow:        
            if math.sqrt((self.bot1Pose.x - self.bot2Pose.x)**2 + (self.bot1Pose.y - self.bot2Pose.y)**2) > R2 \
            and math.sqrt((self.bot1Pose.x - self.bot3Pose.x)**2 + (self.bot1Pose.y - self.bot3Pose.y)**2) > R2:
                msg = Bool()
                msg.data = False
                self.pub1.publish(msg)
                self.B1slow = False
            else:
                msg = Bool()
                msg.data = True
                self.pub1.publish(msg)
            
        else:
            if math.sqrt((self.bot1Pose.x - self.bot2Pose.x)**2 + (self.bot1Pose.y - self.bot2Pose.y)**2) < R1 \
            or math.sqrt((self.bot1Pose.x - self.bot3Pose.x)**2 + (self.bot1Pose.y - self.bot3Pose.y)**2) < R1:
                msg = Bool()
                msg.data = True
                self.pub1.publish(msg)
                self.B1slow = True
        
        if self.B2slow:  
            if math.sqrt((self.bot1Pose.x - self.bot2Pose.x)**2 + (self.bot1Pose.y - self.bot2Pose.y)**2) > R2 \
            and math.sqrt((self.bot2Pose.x - self.bot3Pose.x)**2 + (self.bot2Pose.y - self.bot3Pose.y)**2) > R2:
                msg = Bool()
                msg.data = False
                self.pub2.publish(msg)
                self.B2slow = False
            else:
                msg = Bool()
                msg.data = True
                self.pub2.publish(msg)

        else:
            if math.sqrt((self.bot1Pose.x - self.bot2Pose.x)**2 + (self.bot1Pose.y - self.bot2Pose.y)**2) > R1 \
            or math.sqrt((self.bot2Pose.x - self.bot3Pose.x)**2 + (self.bot2Pose.y - self.bot3Pose.y)**2) > R1:
                msg = Bool()
                msg.data = True
                self.pub2.publish(msg)
                self.B2slow = True

        if self.B3slow:
            if math.sqrt((self.bot1Pose.x - self.bot3Pose.x)**2 + (self.bot1Pose.y - self.bot3Pose.y)**2) > R2 \
            and math.sqrt((self.bot2Pose.x - self.bot3Pose.x)**2 + (self.bot2Pose.y - self.bot3Pose.y)**2) > R2:
                msg = Bool()
                msg.data = False
                self.pub3.publish(msg)
                self.B3slow = False
            else:
                msg = Bool()
                msg.data = True
                self.pub3.publish(msg)

        else:
            if math.sqrt((self.bot1Pose.x - self.bot3Pose.x)**2 + (self.bot1Pose.y - self.bot3Pose.y)**2) > R1 \
            or math.sqrt((self.bot2Pose.x - self.bot3Pose.x)**2 + (self.bot2Pose.y - self.bot3Pose.y)**2) > R1:
                msg = Bool()
                msg.data = True
                self.pub3.publish(msg)
                self.B3slow = True

def main(args=None):
    rclpy.init(args=args)
    avoidance = Avoidance()
    rclpy.spin(avoidance)
    avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--R1", type=int, default=20)
    parser.add_argument("--R2", type=int, default=30)

    args, _ = parser.parse_known_args()
    R1 = args.R1
    R2 = args.R2
    main()

    
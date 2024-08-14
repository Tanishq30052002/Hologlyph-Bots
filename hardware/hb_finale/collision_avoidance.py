import math
from multiprocessing import get_logger
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import argparse

class collisionAvoidance(Node):
    
    def __init__(self, r_outer=0, r_inner=0):
        super().__init__(node_name='collision_avoidance')
        
        self.bot1_pose_sub  = self.create_subscription(Pose2D, "/pen1_pose", self.bot1_pose_callback, 1)
        self.bot2_pose_sub  = self.create_subscription(Pose2D, "/pen2_pose", self.bot2_pose_callback, 1)
        self.bot3_pose_sub  = self.create_subscription(Pose2D, "/pen3_pose", self.bot3_pose_callback, 1)
        
        self.bot1_pose=Pose2D()
        self.bot2_pose=Pose2D()
        self.bot3_pose=Pose2D()

        self.bot1_collision_status_pub = self.create_publisher(String, '/collision_status/bot1', 1)
        self.bot2_collision_status_pub = self.create_publisher(String, '/collision_status/bot2', 1)
        self.bot3_collision_status_pub = self.create_publisher(String, '/collision_status/bot3', 1)

        self.bot1_motion_speed = "fast"
        self.bot2_motion_speed = "fast"
        self.bot3_motion_speed = "fast"

        self.bot1_status = String()
        self.bot2_status = String()
        self.bot3_status = String()

        self.timer = self.create_timer(0.1, self.collision_avoidance)

        self.r_outer = r_outer
        self.r_inner = r_inner
    
    def bot1_pose_callback(self, msg):
        self.bot1_pose.x, self.bot1_pose.y, self.bot1_pose.theta = msg.x, msg.y, msg.theta
    
    def bot2_pose_callback(self, msg):
        self.bot2_pose.x, self.bot2_pose.y, self.bot2_pose.theta = msg.x, msg.y, msg.theta
    
    def bot3_pose_callback(self, msg):
        self.bot3_pose.x, self.bot3_pose.y, self.bot3_pose.theta = msg.x, msg.y, msg.theta
    
    def measure_dist(self, pose1, pose2):
        x1, y1 = pose1.x, pose1.y
        x2, y2 = pose2.x, pose2.y

        dist = math.sqrt((x1-x2)**2+(y1-y2)**2)
        return dist
    
    def collision_avoidance(self):
        bot1_status, bot2_status, bot3_status = self.check_bots_status()
        self.get_logger().info(f"bot1_status: {bot1_status}")
        self.get_logger().info(f"bot2_status: {bot2_status}")
        self.get_logger().info(f"bot3_status: {bot3_status}")

        
    def check_bots_status(self):
        bot1_status = "safe"
        bot2_status = "safe"
        bot3_status = "safe"

        dist_bot1_bot2 = self.measure_dist(self.bot1_pose, self.bot2_pose)
        dist_bot1_bot3 = self.measure_dist(self.bot1_pose, self.bot3_pose)
        dist_bot2_bot3 = self.measure_dist(self.bot2_pose, self.bot3_pose)
        
        # self.get_logger().info(f"Dist Bot - 1&2: {dist_bot1_bot2}")
        # self.get_logger().info(f"Dist Bot - 1&3: {dist_bot1_bot3}")
        # self.get_logger().info(f"Dist Bot - 2&3: {dist_bot2_bot3}")

        # Between Bot1 and Bot2
        if dist_bot1_bot2 > R_outer:
            self.get_logger().info("Safe Zone 1&2")
        elif R_outer > dist_bot1_bot2 > R_inner:
            self.get_logger().info("Collision Potential 1&2")
            if bot1_status != "danger":
                bot1_status = "unsafe"
            if bot2_status != "danger":
                bot2_status = "unsafe"
        elif R_inner > dist_bot1_bot2:
            self.get_logger().info("Danger 1&2")
            bot1_status = "danger"
            bot2_status = "danger"

        # Between Bot1 and Bot3
        if dist_bot1_bot3 > R_outer:
            self.get_logger().info("Safe Zone 1&3")
        elif R_outer > dist_bot1_bot3 > R_inner:
            self.get_logger().info("Collision Potential 1&3")
            if bot1_status != "danger":
                bot1_status = "unsafe"
            if bot3_status != "danger":
                bot3_status = "unsafe"
        elif R_inner > dist_bot1_bot3:
            self.get_logger().info("Danger 1&3")
            bot1_status = "danger"
            bot3_status = "danger"

        # Between Bot2 and Bot3
        if dist_bot2_bot3 > R_outer:
            self.get_logger().info("Safe Zone 2&3")
        elif R_outer > dist_bot2_bot3 > R_inner:
            self.get_logger().info("Collision Potential 2&3")
            if bot2_status != "danger":
                bot2_status = "unsafe"
            if bot3_status != "danger":
                bot3_status = "unsafe"
        elif R_inner > dist_bot2_bot3:
            self.get_logger().info("Danger 2&3")
            bot2_status = "danger"
            bot3_status = "danger"

        self.bot1_status.data=bot1_status
        self.bot2_status.data=bot2_status
        self.bot3_status.data=bot3_status

        self.bot1_collision_status_pub.publish(self.bot1_status)
        self.bot2_collision_status_pub.publish(self.bot2_status)
        self.bot3_collision_status_pub.publish(self.bot3_status)
        # self.get_logger().info(f"Bot1 Status: {bot1_status}")
        # self.get_logger().info(f"Bot2 Status: {bot3_status}")
        # self.get_logger().info(f"Bot3 Status: {bot3_status}")

        return bot1_status, bot2_status, bot3_status
        
def main(args=None):
    rclpy.init(args=args)

    collision_avoidance = collisionAvoidance(r_outer=R_outer, r_inner=R_inner)
    rclpy.spin(collision_avoidance)
    collision_avoidance.destroy_node()

    rclpy.shutdown()

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--R_inner", type=int, default=40)
    parser.add_argument("--R_outer", type=int, default=50)

    args, _ = parser.parse_known_args()
    R_inner = args.R_inner
    R_outer = args.R_outer
    main()
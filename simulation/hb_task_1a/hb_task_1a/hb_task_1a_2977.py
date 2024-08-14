########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1A ###########################################
# Team ID:2977
# Team Leader Name:Pranay Pandey
# Team Members Name:Aman Pandey, Pranay Pandey, Tanishq Chaudhary
# College:IIT (ISM) Dhanbad
########################################################################################################################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen

#Global Variables
KeepRunningTurtle=True

class HologlyphBot(Node):
    def __init__(self, namespace, omega, radius):
        super().__init__(node_name='motion_node', namespace=namespace)

        self.pen_client=self.create_client(SetPen, 'set_pen')

        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(namespace + ' Pen Service not available, waiting...')

        self.req = SetPen.Request()
      
        self.omega=omega
        self.radius=radius

        self.pose_subscriber=self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.pose=Pose()

        self.twist_publisher=self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist=Twist()

        self.timer=self.create_timer(0.001, self.controller)

        self.start_pose=None
        self.end_pose=None

    def pose_callback(self, pose_msg):
        pose_msg.x=round(pose_msg.x, 2)
        pose_msg.y=round(pose_msg.y, 2)
        pose_msg.theta=round(pose_msg.theta, 2)
        self.pose=pose_msg

        self.start_pose = self.start_pose if self.start_pose is not None else self.pose

    def twist_publisher_function(self):
        self.twist.linear.x=self.radius*self.omega
        self.twist.angular.z=self.omega
        self.twist_publisher.publish(self.twist)


    def controller(self):
        error=0.1

        self.circle_check=False
        if self.start_pose is not None and self.pose.x<self.start_pose.x:
            self.circle_check=True

        if self.circle_check and (abs(self.pose.x-self.start_pose.x)<error and abs(self.pose.y-self.start_pose.y)<error):
            global KeepRunningTurtle
            KeepRunningTurtle=False
            self.end_pose=self.end_pose if self.end_pose is not None else self.pose

        self.twist_publisher_function()
    
    def set_pen_properties(self, r, g, b, width, off):
        self.req.r = r
        self.req.g = g
        self.req.b = b
        self.req.width = width
        self.req.off = off
        future = self.pen_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Set pen properties for {self.get_namespace()}')
        else:
            self.get_logger().error(f'Failed to set pen properties')

class SpawnNode(Node):
    def __init__(self, x, y, theta, name):
        super().__init__('spawn_turtle_node') # type: ignore

        # Create a client to the spawn service
        self.spawn_client = self.create_client(Spawn, '/spawn')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Create a request to spawn a new turtle
        self.spawn_request = Spawn.Request()
        self.spawn_request.x = x
        self.spawn_request.y = y
        self.spawn_request.theta = theta
        self.spawn_request.name = name

        # Call the spawn service
        self.future = self.spawn_client.call_async(self.spawn_request)
        self.future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        if future.result() is not None:
            self.get_logger().info(f"Successfully spawned a new turtle: {self.spawn_request.name}")
        else:
            self.get_logger().error('Failed to spawn a new turtle')

def main(args=None):
    global KeepRunningTurtle
    rclpy.init(args=args)

    #Create Node for Turtle1
    Turtle1=HologlyphBot(namespace='turtle1', omega=0.5, radius=1.0)
    Turtle1.set_pen_properties(154, 155, 226, 2, 0)

    while KeepRunningTurtle:
        rclpy.spin_once(Turtle1)

    Turtle1.radius=0.0
    Turtle1.omega=0.0
    rclpy.spin_once(Turtle1)

    Turtle1.get_logger().info("Turtle1: Reached Destination")
    Turtle1.destroy_node()

    end_pose=Turtle1.end_pose
    KeepRunningTurtle=True

    #Spawn Turtle2
    SpawnTurtle2=SpawnNode(x=end_pose.x, y=end_pose.y, theta=end_pose.theta, name='turtle2')
    rclpy.spin_once(SpawnTurtle2)

    #Create Node for Turtle2
    Turtle2=HologlyphBot(namespace='turtle2', omega=-0.5, radius=-2.0)
    Turtle2.set_pen_properties(154, 155, 226, 10, 0)
    while KeepRunningTurtle:
        rclpy.spin_once(Turtle2)

    Turtle2.radius=0.0
    Turtle2.omega=0.0

    rclpy.spin_once(Turtle2)
    Turtle2.get_logger().info("Turtle2: Reached Destination")

    Turtle2.destroy_node()

if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit
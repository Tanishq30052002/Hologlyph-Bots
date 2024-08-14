import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import cv2 as cv
import cv_bridge
import numpy as np
import math

import sys 

debug = False
print_loc = False

# Check if "debug" is provided as a command-line argument
if "debug" in sys.argv:
    debug = True

# Check if "print_loc" is provided as a command-line argument
if "print_loc" in sys.argv:
    print_loc = True

if debug:
    marker_size=2
    margin = 2

class camera_node(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.image_subscription = self.create_subscription(Image, '/image_rect_color', self.image_callback, 10)
        
        self.hb_bot1_pose_publisher = self.create_publisher(Pose2D, '/pen1_pose', 10)
        self.hb_bot2_pose_publisher = self.create_publisher(Pose2D, '/pen2_pose', 10)
        self.hb_bot3_pose_publisher = self.create_publisher(Pose2D, '/pen3_pose', 10)
        self.Robot1_Pose = Pose2D()
        self.Robot2_Pose = Pose2D()
        self.Robot3_Pose = Pose2D()

        self.bot1_loc = [(0, 0)]
        self.bot2_loc = [(0, 0)]
        self.bot3_loc = [(0, 0)]
        
        self.image_publisher = self.create_publisher( Image, '/debug_image', 10)
        self.cv_bridge = cv_bridge.CvBridge()

        self.arena_corner_IDS = np.array([8, 10, 4, 12])
        self.arena_corners = np.full(4, None)

    def image_callback(self, msg):
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.aruco_detection(self.cv_image)

        except Exception as e:
            self.get_logger().error('Error converting ROS Image to OpenCV image: %s' % str(e))
            return

    def dist(self, curr_point_dict, prev_point_tuple):
        if ((curr_point_dict.x)-(prev_point_tuple[0]))**2+((curr_point_dict.y)-(prev_point_tuple[1]))**2 < margin:
            return False
        return True
    
    def aruco_detection(self, frame):
        cv_image = frame

        #Arena Detection
        if np.any(np.equal(self.arena_corners, None)):
            ###
            dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
            parameters = cv.aruco.DetectorParameters()
            detector = cv.aruco.ArucoDetector(dictionary, parameters)
            markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(cv_image)

            cv.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds, (0, 255, 0))
            
            self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

            markerIds =np.squeeze(markerIds)
            markerCorners = np.squeeze(markerCorners)
            ###
            for markerID in markerIds:
                if np.isin(markerID, self.arena_corner_IDS):
                    corner_index=np.where(self.arena_corner_IDS == markerID)[0]
                    marker_index=np.where(markerIds == markerID)[0]
                    if(corner_index==0):
                        point=markerCorners[marker_index[0]][0]
                    if(corner_index==1):
                        point=markerCorners[marker_index[0]][1]
                    if(corner_index==2):
                        point=markerCorners[marker_index[0]][3]
                    if(corner_index==3):
                        point=markerCorners[marker_index[0]][2]
                    self.arena_corners[corner_index[0]]=(point[0], point[1])

        #Perspective Transform
        if not np.any(np.equal(self.arena_corners, None)):
            cv_image=self.perspective_transform(cv_image)
            dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
            parameters =  cv.aruco.DetectorParameters()
            detector = cv.aruco.ArucoDetector(dictionary, parameters)
            self.markerCorners, self.markerIds, rejectedCandidates = detector.detectMarkers(cv_image)

            cv.aruco.drawDetectedMarkers(cv_image, self.markerCorners, self.markerIds, (0, 255, 0))

            # self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

            if self.markerIds is not None:
                for markerID in self.markerIds:
                    if markerID[0] == 1:
                        x, y, theta = self.getPose(markerID[0])
                        self.Robot1_Pose.x = x
                        self.Robot1_Pose.y = y
                        self.Robot1_Pose.theta = theta if(theta<math.pi) else theta - 2*math.pi
                        self.hb_bot1_pose_publisher.publish(self.Robot1_Pose)
                        
                        if debug and self.dist(self.Robot1_Pose, self.bot1_loc[-1]):
                            self.bot1_loc.append((int(x), int(y)))
                    
                    elif markerID[0] == 2:
                        x, y, theta = self.getPose(markerID[0])
                        self.Robot2_Pose.x = x
                        self.Robot2_Pose.y = y
                        self.Robot2_Pose.theta = theta if(theta<math.pi) else theta - 2*math.pi
                        self.hb_bot2_pose_publisher.publish(self.Robot2_Pose)
                        
                        if debug and self.dist(self.Robot2_Pose, self.bot2_loc[-1]):
                            self.bot2_loc.append((int(x), int(y)))

                    elif markerID[0] == 3:
                        x, y, theta = self.getPose(markerID[0])
                        self.Robot3_Pose.x = x
                        self.Robot3_Pose.y = y
                        self.Robot3_Pose.theta = theta if(theta<math.pi) else theta - 2*math.pi
                        self.hb_bot3_pose_publisher.publish(self.Robot3_Pose)
                        
                        if debug and self.dist(self.Robot3_Pose, self.bot3_loc[-1]):
                            self.bot3_loc.append((int(x), int(y)))
        
        if print_loc:
            self.get_logger().info(f"Bot1: x: {self.Robot1_Pose.x}, y: {self.Robot1_Pose.y}, theta: {self.Robot1_Pose.theta}")
            self.get_logger().info(f"Bot2: x: {self.Robot2_Pose.x}, y: {self.Robot2_Pose.y}, theta: {self.Robot2_Pose.theta}")
            self.get_logger().info(f"Bot3: x: {self.Robot3_Pose.x}, y: {self.Robot3_Pose.y}, theta: {self.Robot3_Pose.theta}")
            self.get_logger().info("#"*50)            

        if debug:
            for point in self.bot1_loc:
                cv.circle(cv_image, point, marker_size, (255, 0, 0), -1)
            # Draw all bot2 points on the image with a pink
            for point in self.bot2_loc:
                cv.circle(cv_image, point, marker_size, (0, 255, 0), -1)
            # Draw all bot3 points on the image with a green
            for point in self.bot3_loc:
                cv.circle(cv_image, point, marker_size, (0, 0, 255), -1)

            cv.imshow("Arena Debugger", cv_image)
            cv.waitKey(1)
            # self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        
        else:
            cv.imshow("Arena", cv_image)
            cv.waitKey(1)
            # self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

    def perspective_transform(self, frame):
        cv_image=frame
        pts1=np.array([np.array(point) for point in self.arena_corners])
        pts2 = np.float32([[0, 0], [500, 0], [0, 500], [500, 500]])

        matrix = cv.getPerspectiveTransform(pts1, pts2)
        cv_image = cv.warpPerspective(cv_image, matrix, (500, 500))
        return cv_image
    
    def getPose(self, markerID):
        index = np.where(self.markerIds == markerID)[0][0]
        markerCorner = self.markerCorners[index][0]

        x1, y1 = markerCorner[0]
        x2, y2 = markerCorner[1]
        x3, y3 = markerCorner[2]
        x4, y4 = markerCorner[3]

        if y1>y2:
            theta = np.arccos((x2-x1)/np.sqrt((x2-x1)**2+(y2-y1)**2))
        else:
            theta = 2*math.pi - np.arccos((x2-x1)/np.sqrt((x2-x1)**2+(y2-y1)**2))
        
        x=float(x3+ (1.6/8)*(x3-x4))
        y=float(y3+(3/8)*(y3-y2))
        
        return x, y, theta

def main(args=None):
    rclpy.init(args=args)
    output = camera_node()
    rclpy.spin(output)
    output.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
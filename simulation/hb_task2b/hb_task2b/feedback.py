#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import math

# Import the required modules
##############################################################
class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('ar_uco_detector')

        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        
        self.hb_bot1_pose_publisher = self.create_publisher(Pose2D, '/detected_aruco_1', 10)
        self.hb_bot2_pose_publisher = self.create_publisher(Pose2D, '/detected_aruco_2', 10)
        self.hb_bot3_pose_publisher = self.create_publisher(Pose2D, '/detected_aruco_3', 10)
        self.Robot1_Pose = Pose2D()
        self.Robot2_Pose = Pose2D()
        self.Robot3_Pose = Pose2D()
        
        ### For Debugging
        self.image_publisher = self.create_publisher(Image, '/detected_aruco_image', 10)
        ###
        
    def getPose(self, markerID):
        index = np.where(self.markerIds == markerID)[0][0]
        markerCorner = self.markerCorners[index][0]

        x1, y1 = markerCorner[0]
        x2, y2 = markerCorner[1]
        x3, y3 = markerCorner[2]
        x4, y4 = markerCorner[3]

        x = (x1 + x2 + x3 + x4) / 4
        y = (y1 + y2 + y3 + y4) / 4

        if y1>y2:
            theta = np.arccos((x2-x1)/np.sqrt((x2-x1)**2+(y2-y1)**2))
        else:
            theta = 2*math.pi - np.arccos((x2-x1)/np.sqrt((x2-x1)**2+(y2-y1)**2))
        
        return x, y, theta

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        parameters =  cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(dictionary, parameters)
        self.markerCorners, self.markerIds, rejectedCandidates = detector.detectMarkers(cv_image)

        cv.aruco.drawDetectedMarkers(cv_image, self.markerCorners, self.markerIds, (0, 255, 0))
        
        ### For Debugging
        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        ###

        self.markerIds =np.squeeze(self.markerIds)

        for markerID in self.markerIds:
            if markerID == 1:
                x, y, theta = self.getPose(markerID)
                self.Robot1_Pose.x = x
                self.Robot1_Pose.y = y
                self.Robot1_Pose.theta = theta
                self.hb_bot1_pose_publisher.publish(self.Robot1_Pose)

            elif markerID == 2:
                x, y, theta = self.getPose(markerID)
                self.Robot2_Pose.x = x
                self.Robot2_Pose.y = y
                self.Robot2_Pose.theta = theta
                self.hb_bot2_pose_publisher.publish(self.Robot2_Pose)

            elif markerID == 3:
                x, y, theta = self.getPose(markerID)
                self.Robot3_Pose.x = x
                self.Robot3_Pose.y = y
                self.Robot3_Pose.theta = theta
                self.hb_bot3_pose_publisher.publish(self.Robot3_Pose)

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

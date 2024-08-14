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

import numpy as np
import math

import cv2 as cv
from cv_bridge import CvBridge

# Import the required modules
##############################################################
class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('ar_uco_detector')

        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.aruco_publisher = self.create_publisher(Pose2D, '/detected_aruco', 10)

        self.bridge = CvBridge()

        self.image_publisher = self.create_publisher(Image, '/detected_aruco_image', 10)

        self.Robot1_Pose = Pose2D()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        parameters =  cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(dictionary, parameters)
        markerCorners, markerIds, rejectedCandidates =detector.detectMarkers(cv_image)
        
        cv.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds, (0, 255, 0))
        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

        Robot1_ID=1

        markerIds = np.array(markerIds).squeeze()
        if Robot1_ID in markerIds:
            index = np.where(markerIds == Robot1_ID)[0][0]
            corners = markerCorners[index][0]
            x1, y1 = corners[0]
            x2, y2 = corners[1]
            x3, y3 = corners[2]
            x4, y4 = corners[3]

            self.Robot1_Pose.x = (x1+x2+x3+x4)/4
            self.Robot1_Pose.y = (y1+y2+y3+y4)/4
            
            if y1>y2:                           # In 1st and 2nd quadrant
                self.Robot1_Pose.theta = np.arccos((x2-x1)/np.sqrt((x2-x1)**2+(y2-y1)**2))
            else:                               # In 3rd and 4th quadrant
                self.Robot1_Pose.theta = 2*math.pi - np.arccos((x2-x1)/np.sqrt((x2-x1)**2+(y2-y1)**2))
            
        else:
            print('Robot1 not found')

        self.aruco_publisher.publish(self.Robot1_Pose)

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

# ! /usr/bin/env python3
import rospy
import numpy as np 
import imutils
import cv2
import math

import random


def random_color():
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    return (r, g, b)


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)


# Image size (width, height)
image_sz = (840, 960)

# Focal length in pixels
focal_length = 500.0

# Principal point coordinates in pixels (usually at the image center)
principal_pt = (image_sz[0] / 2.0, image_sz[1] / 2.0)

# Camera matrix

# Distortion coefficients (all zeros for an ideal camera)


class detection():
    def __init__(self):

        self.center = None
        self.markerID = None
        self.radius = None
        self.current_id = 0
        # simulation marker size in meters
        self.markerSize = 0.168
        self.aruco_list = []
        # matrices for simulator camera
        # self.mtx = np.array([[1797.29739, 0.0, 965.41559],
        #        [0.0, 1798.60416, 519.75109],
        #        [0.0, 0.0, 1.0]],
        #        dtype=np.float32)
        
        #alt name - cameraMatrix
        self.mtx = np.array([
        [focal_length, 0.0, principal_pt[0]],
        [0.0, focal_length, principal_pt[1]],
        [0.0, 0.0, 1.0],],
        dtype=np.float32)

        # self.dist = np.array([[-0.003279, 0.025297, -0.003146, 0.001936, 0.00]],
        #         dtype=np.float32)
        # alt name - distCoeffs
        self.dist = np.zeros((4, 1), dtype=np.float32)

    def aruco_detection(self, image):
        self.aruco_list.clear()
        self.image = image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        (corners, ids, rejected) = detector.detectMarkers(image)

        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        if len(corners) > 0:
            ids = ids.flatten()

            for (markerCorner, detected_markerID) in zip(corners, ids):


                int_corners = markerCorner.reshape((4, 2))
                int_corners = int_corners.astype(int)
                (topLeft, topRight, bottomRight, bottomLeft) = int_corners
                
                rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.markerSize, self.mtx, self.dist)
                
                # 3D axes drawing
                cv2.drawFrameAxes(image, self.mtx,  self.dist , rvec, tvec, 0.1);
                # distance from robot to detected aruco
                if len(tvec) > 0:
                    self.dist = round(tvec[0][0][2],1)
                else:
                    self.dist = None

                r = int(math.sqrt(
                    (int(topRight[0]) - int(bottomLeft[0])) ** 2 + (int(topRight[1]) - int(bottomLeft[1])) ** 2) / 2)
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                self.center = (cX, cY)
                # cv2.putText(image, self.center, cv2.FONT_HERSHEY_SIMPLEX, 0.2, random_color(), 0.3, cv2.LINE_AA)

                # cv2.circle(image, (cX, cY), radius, (0, 0, 255), 3)

                cv2.putText(image, f"dist {self.dist}",
                            (cX, cY-100),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 0), 2)

                self.markerID = detected_markerID
                self.radius = r

                self.aruco_list.append( {"center":self.center, 
                        "radius":self.radius, "markerID":self.markerID, "dist":self.dist} )
               
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            exit()

        # sort by marker id before return it
        if len(self.aruco_list) > 0: return [self.image, sorted(self.aruco_list, key=lambda x: x["markerID"]) ]
        else: return [self.image, self.aruco_list]
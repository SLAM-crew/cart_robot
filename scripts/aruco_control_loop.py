#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from aruco_detect import detection
from cv_bridge import CvBridge, CvBridgeError

import cv2
from sensor_msgs.msg import Image



class Robot_Controller:
    def __init__(self):
        
        self.image_sz = (840, 960)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cart_robot/camera_main/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("/aruco_processing", Image,  queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity_msg = Twist()

        self.angular_p = 0.005
        self.linear_p = 0.009
        self.radius_threshold = 150
        self.theta_precision = 40
        
        self.current_id = 0
        self.MAX_ID = 3

        
        self.detect = detection() 

        self.lt = ""
        self.at = ""

    def move(self, linear, angular):
        self.velocity_msg.linear.x = -1 * linear
        self.velocity_msg.angular.z = angular
        rospy.loginfo(f"linear velocity: {linear}")
        rospy.loginfo(f"angular velocity: {angular}")

        self.pub.publish(self.velocity_msg)

    def callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.control_loop()
        except CvBridgeError as e:
            print(e)

    def direction(self, hunting):

        if hunting == 1:
            return 0.3, "Turning Left ", "ID - 1"
        elif hunting == 2:
            return -0.3, "Turning Right", "ID - 2"
        elif hunting == 3:
            return 0, "", "Parked" , "ID - 3"

    def control_loop(self):

        self.detected_aruco = self.detect.aruco_detection(self.image)

        if len(self.detected_aruco[1]) > 0:
            for aruco in self.detected_aruco[1]:
                if self.current_id + 1 == aruco["markerID"]:
                    if aruco["dist"] < 1:
                        rospy.loginfo("CHANGING DIRECTION")
                        self.current_id += 1
                        angular = self.direction(self.current_id)
    
                    arucoX = aruco['center'][0]

                    self.theta_error = self.image_sz[1] / 2 - arucoX

                    rospy.loginfo(f"total detected:{len(self.detected_aruco[1])}")    
                    rospy.loginfo(f"Hunting for: {aruco['markerID']}")
                    rospy.loginfo("current id: " + str(self.current_id))
                    rospy.loginfo(f"aruco position, x: {arucoX}")
                    rospy.loginfo(f"raduis {aruco['radius']}")
                    rospy.loginfo("theta error: " + str(self.theta_error) + "\n")

                    if aruco["radius"] < self.radius_threshold:
                        if self.theta_error > 0 and (self.theta_precision < abs(self.theta_error)):
                            self.move(self.linear_p * (self.radius_threshold - aruco["radius"]),
                                    self.angular_p * self.theta_error )

                            self.at = " <-- LEFT"
                            self.lt = "Move Forward"


                        elif self.theta_error < 0 and (self.theta_precision < abs(self.theta_error)):
                            self.move(self.linear_p * (self.radius_threshold - aruco["radius"]),
                                    self.angular_p * self.theta_error)
                            self.at = "  RIGHT-->"
                            self.lt = "Move Forward"

                        else:
                            self.move(self.linear_p * (self.radius_threshold - aruco["radius"]), 0)
                            self.at = " CENTER "
                            self.lt = "Move Forward"
                    else:

                        if self.theta_error > 0 and (self.theta_precision > abs(self.theta_error)):
                            self.move(0, self.angular_p * self.theta_error)
                            self.at = "<-- LEFT"
                            self.lt = "Stop"
                        elif self.theta_error < 0 and (self.theta_precision > abs(self.theta_error)):
                            self.move(0, self.angular_p * self.theta_error)
                            self.at = " RIGHT-->"
                            self.lt = "Stop"
                        else:
                            angular = self.direction(self.detect.markerID)
                            self.lt = angular[2]
                            self.at = angular[1]
                            self.move(0, angular[0])

                 
            cv2.putText(self.detected_aruco[0], self.lt, (300, 800), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(self.detected_aruco[0], self.at, (350, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)


        else:
            self.move(0,0.4)
            self.at = "Finding Aruco"

        if self.MAX_ID == self.current_id:
            self.move(0,0)
            exit()
        
        
        cv2.imshow("Frame", self.detected_aruco[0])
        cv2.waitKey(1)


def main():
    rospy.init_node("robot_aruco_controller", anonymous=True)
    of = Robot_Controller()
    try:
        rospy.spin()

    except:

        print("error")
    cv2.destroyAllWindows()


main()



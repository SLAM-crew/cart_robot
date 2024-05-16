#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class CartRobotControl:
    def __init__(self):
        rospy.loginfo("[Solution] started loaging")

        rospy.init_node('lidar_processing_node', anonymous=True)
        rospy.Subscriber("/cart_robot/lidar_main", LaserScan, self.lidar_callback)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.joint_speed = 0.08  

        rospy.loginfo("[Solution] loaded")
    
    # setted random values and directions in move_forward & turn_left & turn_right
    # needed to be calibrated
    # commented lidar_callback --> because debugging via teleop
    def move_forward(self):
        self.command.linear.x = self.joint_speed
        self.command.linear.y = 0.0
        self.command.angular.z = 0.0
        self.cmd_vel.publish(self.command)

    def turn_left(self):
        self.command.linear.x = self.joint_speed
        self.command.linear.y = 0.0
        self.command.angular.z = 2.5
        self.cmd_vel.publish(self.command)
    
    def turn_right(self):
        self.command.linear.x = self.joint_speed
        self.command.linear.y = 0.0
        self.command.angular.z = -2.5
        self.cmd_vel.publish(self.command)

    def lidar_callback(self, data):
        pass
        # distance_threshold = 4.0  
        # if data.ranges[len(data.ranges) // 2] < distance_threshold:
        #     rospy.loginfo("Object detected in laser zone!")
        #     self.turn_left()
        # else:
        #     rospy.loginfo("No object detected in laser zone.")
        #     self.move_forward()
            
def main():
    try:
        CartRobotControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
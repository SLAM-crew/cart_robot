#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

class CartRobotControl:
    def __init__(self):
        rospy.init_node('lidar_processing_node', anonymous=True)
        rospy.Subscriber("/cart_robot/lidar_main", LaserScan, self.lidar_callback)

        self.up_right_wheel_controller = rospy.Publisher('/cart_robot/up_right_wheel_controller/command', Float64, queue_size=1)
        self.up_left_wheel_controller = rospy.Publisher('/cart_robot/up_left_wheel_controller/command', Float64, queue_size=1)

        self.down_right_wheel_controller = rospy.Publisher('/cart_robot/down_right_wheel_controller/command', Float64, queue_size=1)
        self.down_left_wheel_controller = rospy.Publisher('/cart_robot/down_left_wheel_controller/command', Float64, queue_size=1)

    def move_forward(self):
        joint_speed = 5.0  
        self.up_right_wheel_controller.publish(-joint_speed)
        self.up_left_wheel_controller.publish(-joint_speed)

        self.down_right_wheel_controller.publish(-joint_speed)
        self.down_left_wheel_controller.publish(-joint_speed)

    def turn_left(self):
        joint_speed = 4.0  
        self.up_right_wheel_controller.publish(-joint_speed)
        self.down_right_wheel_controller.publish(-joint_speed)
        self.up_left_wheel_controller.publish(joint_speed)
        self.down_left_wheel_controller.publish(joint_speed)

    def lidar_callback(self, data):
        distance_threshold = 4.0  
        if data.ranges[len(data.ranges) // 2] < distance_threshold:
            rospy.loginfo("Object detected in laser zone!")
            self.turn_left()
        else:
            rospy.loginfo("No object detected in laser zone.")
            self.move_forward()
            
def main():
    try:
        CartRobotControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
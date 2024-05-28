#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped

class ConvertMsgs:
    def __init__(self):
        rospy.init_node('convert_msgs_node', anonymous=True)
        rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.pose_callback)
        self.pose_pub = rospy.Publisher("/cart_robot/pose", PoseWithCovarianceStamped, queue_size=3)

    def pose_callback(self, data):
        pose = PoseWithCovarianceStamped()
        pose.header = data.header
        poseCov = PoseWithCovariance()
        poseCov.pose = data.pose
        pose.pose = poseCov
        self.pose_pub.publish(pose)
            
def main():
    try:
        ConvertMsgs()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
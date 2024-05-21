#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
import move_base_msgs.msg
import actionlib
import actionlib_msgs.msg

# code in progress, not sure its working
class NavigationController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("navigation_controller")

        # Create a listener for the robot's base frame
        self.tf_listener = tf.TransformListener()

        # Create a publisher for the robot's goal pose
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=10)

        # Create a subscriber for the robot's current pose
        self.pose_subscriber = rospy.Subscriber("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped, self.on_pose_received)

        # Create a client for the move_base action server
        self.move_base_client = actionlib.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)

        # Wait for the move_base action server to become available
        self.move_base_client.wait_for_server()

    def on_pose_received(self, pose_msg):
        # Extract the robot's current pose from the message
        pose = pose_msg.pose.pose

        # Convert the pose to the map frame
        try:
            map_pose = self.tf_listener.transformPose("map", pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # Create a goal pose message in the map frame
        goal_msg = geometry_msgs.msg.PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose = map_pose.pose

        # Send the goal pose message to the move_base action server
        self.move_base_client.send_goal(goal_msg)

        # Wait for the move_base action to complete
        self.move_base_client.wait_for_result()

        # Check the result of the move_base action
        result = self.move_base_client.get_result()
        if result.status == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached goal pose")
        else:
            rospy.logerr("Failed to reach goal pose")

if __name__ == "__main__":
    try:
        controller = NavigationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

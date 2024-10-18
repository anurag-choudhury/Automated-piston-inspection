#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal():
    # Initialize the ROS node
    rospy.init_node('amr_navigation_script', anonymous=True)

    # Create an action client to send goals to the move_base server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to start
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server")

    # Define the goal position and orientation
    goal = MoveBaseGoal()

    # Set frame_id to 'map' so the goal is in the global map frame
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set goal position (x, y, z)
    goal.target_pose.pose.position.x = 5.605790138244629
    goal.target_pose.pose.position.y = 7.377140045166016
    goal.target_pose.pose.position.z = 0.0

    # Set goal orientation (x, y, z, w)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.44766193306161234
    goal.target_pose.pose.orientation.w = 0.894202881726256

    rospy.loginfo("Sending goal to AMR...")

    # Send the goal
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    # Check if the robot successfully reached the goal
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("The robot failed to reach the goal.")

if __name__ == '__main__':
    try:
        move_to_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")

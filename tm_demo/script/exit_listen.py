#!/usr/bin/env python3

import rospy
from tm_msgs.srv import SendScript, SendScriptRequest, SendScriptResponse

# Initialize the ROS node
rospy.init_node('exit_listen_node')

# Service proxy for sending script to the Techman robot
send_script_service = rospy.ServiceProxy('/tm_driver/send_script', SendScript)

def exit_listen_node():
    """Send the command to exit the listen node."""
    rospy.loginfo("Sending exit listen node command...")
    script = "ScriptExitListen()"  # Replace with the actual TMFlow script command to exit the listen node
    try:
        req = SendScriptRequest()
        req.script = script
        res = send_script_service(req)
        if res.ok:
            rospy.loginfo("Successfully exited listen node.")
        else:
            rospy.logerr("Failed to exit listen node.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.wait_for_service('/tm_driver/send_script')  # Wait until the service is available
    exit_listen_node()

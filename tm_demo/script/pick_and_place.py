#!/usr/bin/env python3
import subprocess
import rospy
from geometry_msgs.msg import PoseStamped
from tm_msgs.srv import SetPositions, SetPositionsRequest
import tf

# Initialize global variables
position_set = False
subscriber = None  # Declare the subscriber variable globally

def exitListen():
    script = "bash -c 'source /fsm_piston/devel/setup.bash && rosrun demo demo_leave_listen_node'"
    subprocess.run(script, shell=True)

def isListenActive():
    result = subprocess.run(
        ['rosrun', 'demo', 'demo_ask_sta'],  # Command to run
        stdout=subprocess.PIPE,               # Capture the standard output
        stderr=subprocess.PIPE,               # Capture the error output
        text=True                             # Return output as string (Python 3.7+)
    )
    return "subdata is true" in result.stdout

def pose_callback(data):
    global position_set  # Use the global variable to track the state

    if not position_set:
        rospy.loginfo("Received cobot pose from TMFlow: %s", data.pose)
        
        # Extract position and calculate new position
        x = data.pose.position.x-0.0009
        # y = data.pose.position.y        # Applying offset in Y
        # z = data.pose.position.z
        y = data.pose.position.y -0.1327    # Applying offset in Y
        z = data.pose.position.z-0.0249

        # Extract orientation (quaternion) from the pose
        # quaternion = (
        #     data.pose.orientation.x ,
        #     data.pose.orientation.y ,
        #     data.pose.orientation.z ,
        #     data.pose.orientation.w 
        # )
        quaternion = (
            data.pose.orientation.x -0.00333,
            data.pose.orientation.y -0.01301,
            data.pose.orientation.z + 0.00492,
            data.pose.orientation.w + 0.00350
        )

        # Convert quaternion to roll, pitch, and yaw
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

        # New position with (x, y, z) and (roll, pitch, yaw)
        new_pos = [x, y, z, roll, pitch, yaw]

        # Move the robot to the new position
        move_robot_to_position(new_pos)
        exitListen()
        # Set the flag to prevent further calls
        position_set = True

        # Unsubscribe from the topic to prevent further callbacks
        rospy.loginfo("Unsubscribing from /tool_pose")
        subscriber.unregister()  # Unregister the subscriber directly

def move_robot_to_position(position):
    rospy.wait_for_service('/tm_driver/set_positions')
    try:
        set_positions = rospy.ServiceProxy('/tm_driver/set_positions', SetPositions)
        pos_request = SetPositionsRequest()
        pos_request.motion_type = 2  # Cartesian motion
        pos_request.positions = position  # [X, Y, Z, Roll, Pitch, Yaw]
        pos_request.velocity = 0.8  # Set the desired speed
        pos_request.fine_goal = True  # Fine goal
        set_positions(pos_request)
        rospy.loginfo(f"Moved robot to position: {position}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def check_listener_state(event):
    global subscriber, position_set  # Use global variables

    if isListenActive():
        if subscriber is None:  # Only subscribe if not already subscribed
            rospy.loginfo("Starting to listen for poses...")
            subscriber = rospy.Subscriber("/tool_pose", PoseStamped, pose_callback)
            position_set = False  # Reset position_set when starting to listen
    else:
        if subscriber is not None:  # Unsubscribe if listener is not active
            rospy.loginfo("Stopping listening for poses...")
            subscriber.unregister()
            subscriber = None  # Reset the subscriber

if __name__ == '__main__':
    subprocess.run("bash -c 'source /fsm_piston/devel/setup.bash'", shell=True)

    rospy.init_node('cobot_pose_listener')

    # Start a timer to check the listener state at regular intervals
    rospy.Timer(rospy.Duration(1.0), check_listener_state)  # Check every 1 second

    rospy.spin()

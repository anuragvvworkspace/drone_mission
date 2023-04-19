#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy
import math
from tf.transformations import quaternion_from_euler

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py2")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 1
    pose.pose.position.y = 1
    pose.pose.position.z = 3
    
    azimuth = math.atan2(0 - self.currPose.pose.position.y, 0 - self.currPose.pose.position.x)
	# print(azimuth)
	if azimuth > math.pi:
		azimuth -= 2.0 * math.pi
	else:
		azimuth += 2.0 * math.pi
	q = quaternion_from_euler(0, 0, azimuth)
	# print(q)
	pose.pose.orientation.x = q[0]
	pose.pose.orientation.y = q[1]
	pose.pose.orientation.z = q[2]
	pose.pose.orientation.w = q[3]
	local_pos_pub.publish(self.desPose)
	self.rate.sleep()

   
    local_pos_pub.publish(pose)
    print("pose piblished==============="

    rate.sleep()


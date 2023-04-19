#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy
import math
from tf.transformations import quaternion_from_euler


class midterm_drone():
	def __init__(self):
		print("Init initi")
		rospy.init_node("offb_node_py")
		self.current_state = State();
		
		self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)

		self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
		
		rospy.wait_for_service("/mavros/cmd/arming")
		arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

		rospy.wait_for_service("/mavros/set_mode")
		set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
		

		# Setpoint publishing MUST be faster than 2Hz
		rate = rospy.Rate(20)
		
		print("About to enter while to connect")
		# Wait for Flight Controller connection
		while(not rospy.is_shutdown() and not self.current_state.connected):
			rate.sleep()
			#print("connect B****! \n",self.current_state)
		#print("current_state, See its connected: ",self.current_state);
		
		self.pose = PoseStamped()

		self.pose.pose.position.x = 0
		self.pose.pose.position.y = 0
		self.pose.pose.position.z = 2

		# Send a few setpoints before starting
		for i in range(10):   
			if(rospy.is_shutdown()):
				break
			print("i loop: ",i)

			self.local_pos_pub.publish(self.pose)
			rate.sleep()

		offb_set_mode = SetModeRequest()
		offb_set_mode.custom_mode = 'OFFBOARD'
		print("Just after offb_set_mode",self.current_state);
		#print("Time ",rospy.Time.now())

		arm_cmd = CommandBoolRequest()
		arm_cmd.value = True

		last_req = rospy.Time.now()
		sec_req = rospy.Time.now()

		while(not rospy.is_shutdown()):
			if((rospy.Time.now() - sec_req) > rospy.Duration(1.0)):
				print("--- In while\n",self.current_state)
				print("Time ",rospy.Time.now() - last_req)
				sec_req = rospy.Time.now()
			if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
				if(set_mode_client.call(offb_set_mode).mode_sent == True):
					rospy.loginfo("OFFBOARD enabled")
				
				last_req = rospy.Time.now()
			else:
				if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
					if(arming_client.call(arm_cmd).success == True):
						rospy.loginfo("Vehicle armed")
					last_req = rospy.Time.now()

			if(self.current_state.mode == "OFFBOARD" and self.current_state.armed):
				print("third IF--------")
				if((rospy.Time.now() - last_req) > rospy.Duration(2.0)):
			#		last_req = rospy.Time.now()
					break;
			self.local_pos_pub.publish(self.pose)
			rate.sleep()
		
		
		self.despose = PoseStamped()
		self.circleAround(0,0,0,3,8);
		
	def getDist(self, curr, des):
		return math.sqrt(pow(curr[0] - des[0], 2) + pow(curr[1] - des[1], 2) + pow(curr[2] - des[2], 2))
				
	def circleAround(self,x,y,z,r,n):
		for i in range(n):
			# Mission Starts here!!!
			self.despose.pose.position.x = x+r*math.cos(i*2*math.pi/n);
			self.despose.pose.position.y = y+r*math.sin(i*2*math.pi/n);
			self.despose.pose.position.z = 4
			
			azimuth = math.atan2(0 - self.despose.pose.position.y, 0 - self.despose.pose.position.x)
			# print(azimuth)
			if azimuth > math.pi:
				azimuth -= 2.0 * math.pi
			else:
				azimuth += 2.0 * math.pi
			q = quaternion_from_euler(0, 0, azimuth)
			# print(q)
			self.despose.pose.orientation.x = q[0]
			self.despose.pose.orientation.y = q[1]
			self.despose.pose.orientation.z = q[2]
			self.despose.pose.orientation.w = q[3]
			while(not rospy.is_shutdown()):
				self.local_pos_pub.publish(self.despose);
				rate.sleep()
			
    

		

	def state_cb(self, msg):
		self.current_state = msg



if __name__ == "__main__":
    highflier = midterm_drone();


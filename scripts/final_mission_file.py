#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy
import math
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import PositionTarget

class midterm_drone():
	def __init__(self):
		self.distThr = 0.5;
		print("Init initi")
		rospy.init_node("offb_node_py")
		self.current_state = State();
		
		self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
		self.pose_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
		
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
		self.despose = PoseStamped()

		self.pose.pose.position.x = 0
		self.pose.pose.position.y = 0
		self.pose.pose.position.z = 2

		# Send a few setpoints before starting
		for i in range(20):   
			if(rospy.is_shutdown()):
				break
			print("i loop: ",i)

			self.pose_pub.publish(self.pose)
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
			if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
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
				if((rospy.Time.now() - last_req) > rospy.Duration(5.0)):
			#		last_req = rospy.Time.now()
					break;
			self.pose_pub.publish(self.pose)
			rate.sleep()
		
		# Mission Starts here!!!
		print("----Mission Starts here!!!----")
		self.drone_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.drone_pose_cb)
		self.rate = rospy.Rate(50)
		self.rate = rospy.Rate(50)
		self.des = []
		self.currPose = PoseStamped()
		self.desPose = PoseStamped()
		self.curr = self.currPose.pose.position
		self.rock_x,self.rock_y, self.rock_z = 60.2,-12.5, 18.8
		#rospy.spin();
		print("------------================= READY to GO =============-----------------")
		

	def state_cb(self, msg):
		self.current_state = msg

	def drone_pose_cb(self, msg):
		self.currPose = msg
		self.curr = self.currPose.pose.position
		# print("Drone pose received", self.curr)

	def getDist(self, curr, des):
		return math.sqrt(pow(curr.x - des[0], 2) + pow(curr.y - des[1], 2) + pow(curr.z - des[2], 2))
		
	def navigate(self, x, y, z ):
		self.desPose.pose.position.x = x
		self.desPose.pose.position.y = y
		self.desPose.pose.position.z = z
		rospy.loginfo("Desired position: x={}, y={}, z={}".format(x, y, z))
		self.des = [x, y, z]
		# print(self.curr)
		d = self.getDist(self.curr, self.des)
		while d > self.distThr and not rospy.is_shutdown():
			# print("Drone pose received", self.curr)
			# self.curr = self.currPose.pose.position
			# print(d, '--' , self.curr, '--' , self.des)
			azimuth = math.atan2(self.rock_y - self.currPose.pose.position.y,self.rock_x - self.currPose.pose.position.x)
			# print(azimuth)
			if azimuth > math.pi:
				azimuth -= 2.0 * math.pi
			else:
				azimuth += 2.0 * math.pi
			q = quaternion_from_euler(0, 0, azimuth)
			# print(q)
			self.desPose.pose.orientation.x = q[0]
			self.desPose.pose.orientation.y = q[1]
			self.desPose.pose.orientation.z = q[2]
			self.desPose.pose.orientation.w = q[3]
			d = self.getDist(self.curr, self.des)
			self.pose_pub.publish(self.desPose)
			self.rate.sleep()
			if d <= self.distThr:
				print(azimuth)
				print(q)
				break
		
	def circleRock(self,x,y,z,r,n,zh):
		print("=== Circle rock ========")
		#r = 8;
		#n = 24;
		init_ang = math.atan2(y - self.curr.y, x - self.curr.x)
		for i in range(n):
			self.despose.pose.position.x = x+r*math.cos(i*2*math.pi/n - init_ang);
			self.despose.pose.position.y = y+r*math.sin(i*2*math.pi/n - init_ang);
			self.despose.pose.position.z = z + zh;
			self.des = [self.despose.pose.position.x, self.despose.pose.position.y, self.despose.pose.position.z];
			#rospy.loginfo("Desired position: x={}, y={}, z={}".format(self.des[0], self.des[1], self.des[2]))
			azimuth = math.atan2(y - self.despose.pose.position.y, x - self.despose.pose.position.x)
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
			rospy.loginfo("Desired position: x={}, y={}, z={}, qx={}, qy = {}, qz = {}, qw = {}".format(self.des[0], self.des[1], self.des[2], q[0], q[1],q[2],q[3]))
			d = self.getDist(self.curr, self.des)
			while d > self.distThr and not rospy.is_shutdown():
				self.pose_pub.publish(self.despose);
				d = self.getDist(self.curr, self.des);
				self.rate.sleep()
				
								
	def gotoSampleprobe(self):
		print("=== Going to probe location ========")
		locations = numpy.matrix([[40.8, 3.5, 20],])
		for waypt in locations:
			x, y, z = waypt.tolist()[0]
			self.navigate(x, y, z)

	def pickProbe(self):
		print("=== Pick probe using visual servoing(currently position control) ========")
		locations = numpy.matrix([[40.8, 3.5, 12.2],[40.8, 3.5, 20]]);
		for waypt in locations:
			x, y, z = waypt.tolist()[0]
			self.navigate(x, y, z)

	def gotoRock(self):
		print("=== GO to rock and Face rock ========")
		locations = numpy.matrix([[60, -15, 18.89],])
		for waypt in locations:
			x, y, z = waypt.tolist()[0]
			self.navigate(x, y, z)
	
	def gotoRover(self):
		print("=== GO to rock and Face rock ========")
		locations = numpy.matrix([[12.62, -64.8, 2],[12.62, -64.85, 0.5]])
		for waypt in locations:
			x, y, z = waypt.tolist()[0]
			self.navigate(x, y, z)

	def land_on_rover(self):
		local_pos_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
		arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		landing_position = PositionTarget()
		landing_position.position.x = 12.621
		landing_position.position.y = -64.85
		landing_position.position.z = -3.5
		landing_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
		landing_position.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
							+ PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
							+ PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE
		# Publish the landing position command
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			local_pos_pub.publish(landing_position)
			rate.sleep()
			if(self.currPose.pose.position.z < -3.2):
				print("Landed =========")
				arm_service(False);

	def run(self):
		#locations = numpy.matrix([[57, -12, 18.89],[60, -15, 18.89],[63, -12, 18.89],[60, -9, 18.89],[57, -12, 18.89]])
		self.gotoSampleprobe();
		self.pickProbe();
		self.gotoRock();
		###self.circleRock(0,-3,3,4,16);
		self.circleRock(self.rock_x,self.rock_y,self.rock_z,3.5,24,1);
		self.gotoRover();
		self.land_on_rover();
	
if __name__ == "__main__":
    highflier = midterm_drone();
    highflier.run();

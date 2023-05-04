	def navigateroverVelctrl(self, x, y ):
		print(1)
		self.desPose.pose.position.x = x
		self.desPose.pose.position.y = y
		self.desPose.pose.position.z = 0
		print(2)
		
		
		#rospy.loginfo("Desired position: x={}, y={}, z={}".format(x, y, 0))
		self.des = [x, y, 0]
		print(3)
		# print(self.curr)
		rospy.sleep(0.1)
		d = self.getDist(self.curr, self.des)
		#print(4,"  d  ", d ,  "  ", )
		cx = self.currPose.pose.position.x;
		cy = self.currPose.pose.position.y;
		cz = self.currPose.pose.position.z;
			
		print("Desired position: x={}, y={}, z={}".format(x, y, 0));
			
		while d > self.distThr and not rospy.is_shutdown():
			#print(5)
			cx = self.currPose.pose.position.x;
			cy = self.currPose.pose.position.y;
			cz = self.currPose.pose.position.z;
			#print(6)
			#print(self.currPose.pose.position.x, " ", self.currPose.pose.position.y , "  ", self.currPose.pose.position.z,"    -navigateroverVelctrl-- ")
			#print(7)
			# Publish rot + vel till azimuth close to zero, then only forward vel
			azimuth = math.atan2(self.desPose.pose.position.y - self.currPose.pose.position.y,self.desPose.pose.position.x - self.currPose.pose.position.x)
			#print(8)
			while azimuth > math.pi or azimuth < -math.pi:
				if azimuth > math.pi:
					azimuth -= 2.0 * math.pi
				else:
					azimuth += 2.0 * math.pi
			#print(9)
			# print("Drone pose received", self.curr)
			self.curr = self.currPose.pose.position
			# print(d, '--' , self.curr, '--' , self.des)
			# print(azimuth)
			
			#q = quaternion_from_euler(0, 0, azimuth)
			# print(q)
			botangles = euler_from_quaternion([self.currPose.pose.orientation.x, self.currPose.pose.orientation.y, self.currPose.pose.orientation.z, self.currPose.pose.orientation.w])
			#print("bot angles : ", botangles)
			botazimuth = botangles[2];
			#print(10)
			#print("Angles : azimuth={}, botazimuth={}".format(azimuth, botazimuth));
			while botazimuth > math.pi or botazimuth < -math.pi:
				if botazimuth > math.pi:
					botazimuth -= 2.0 * math.pi
				else:
					botazimuth += 2.0 * math.pi
			#print(11)
			angerr = azimuth - botazimuth;
			
			vel_cmd = Twist()
			vel_cmd.linear.x = 0.5  # Set linear velocity in the x-axis direction
			#print(12)
			if abs(angerr) < 0.05:
				vel_cmd.angular.z = 0.0  # Set angular velocity around the z-axis
			else:
				if angerr > 0:
					if angerr < math.pi:
						vel_cmd.angular.z = 1.0  # Set angular velocity around the z-axis
					else:
						vel_cmd.angular.z = -1.0  # Set angular velocity around the z-axis
				else:
					if angerr > -math.pi:
						vel_cmd.angular.z = -1.0  # Set angular velocity around the z-axis
					else:
						vel_cmd.angular.z = 1.0  # Set angular velocity around the z-axis
			#print(12)

			# Publish the velocity command
			self.vel_pub.publish(vel_cmd)
			self.rate.sleep()
			d = self.getDist(self.curr, self.des)
			#print(13)
			if d <= self.distThr:
				vel_cmd.linear.x = 0.0
				vel_cmd.angular.z = 0.0
				self.vel_pub.publish(vel_cmd)
				self.rate.sleep()			
				break
			#print(14)
			

#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

import time
from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	error=[]
	prev_err=[0,0,0,0]
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = [-4.07,2.35,28.10,0.01] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [0,0,0,0]
		self.Ki = [0,0,0,0]
		self.Kd = [0,0,0,0]


		#-----------------------Add other required variables for pid here ----------------------------------------------


		
		self.max_values = [1800,1800,1800,1800] 
		self.min_values = [1200,1200,1200,1200]
		self.avg_values = [
			(self.max_values[0]+self.min_values[0])/2,
			(self.max_values[1]+self.min_values[1])/2,
			(self.max_values[2]+self.min_values[2])/2,
			(self.max_values[3]+self.min_values[3])/2
		] 
				
		self.error=[0,0,0,0]
		self.prev_err=[0,0,0,0]
		self.iterm=[0,0,0,0]
		
		self.roll_pitch=0
		self.roll_yaw=0
		self.roll_roll=0
		self.roll_throttle=0





		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		#pubs=["/drone_command", "/alt_error", "/pitch_error", "/roll_error", "/yaw_error"]
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		""""#self.command_pub = rospy.Publisher('/alt_error', PlutoMsg, queue_size=1)
		#self.command_pub = rospy.Publisher('/pitch_error', PlutoMsg, queue_size=1)
		#self.command_pub = rospy.Publisher('/roll_error', PlutoMsg, queue_size=1)
		#self.command_pub = rospy.Publisher('/yaw_error', PlutoMsg, queue_size=1)"""

		"""for i in range(len(pubs)):
			self.command_pub[i] = rospy.Publisher(pubs[i], PlutoMsg, queue_size=1)	"""
		






		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------






		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		time.sleep(2)					# 2 seconds lag
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		self.drone_position[3] = msg.poses[0].position.w
	




		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[0] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[0] = alt.Ki * 0.008
		self.Kd[0] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------

		for i in range(1,4):
			self.Kp[i] = alt.Kp * 0.06
			self.Ki[i] = alt.Ki * 0.008
			self.Kd[i] = alt.Kd * 0.3








	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
	
		for i in range(4):
			self.error[i]=self.drone_position[i] - self.setpoint[i]
		

	# for pitch

		self.iterm[0]=(self.iterm[0]+self.error[0])*self.ki[0]
		self.out_roll=self.Kp[0]*self.error[0]+self.Kd[0]*(self.err[0]-self.prev_err[0])+self.iterm[0]

		# for roll
		
		self.iterm[1]=(self.iterm[1]+self.error[1])*self.ki[1]
		self.out_pitch=self.Kp[1]*self.error[1]+self.Kd[1]*(self.err[1]-self.prev_err[1])+self.iterm[1]

		# for yaw
		
		self.iterm[2]=(self.iterm[2]+self.error[2])*self.ki[2]
		self.out_yaw=self.Kp[2]*self.error[2]+self.Kd[2]*(self.err[2]-self.prev_err[2])+self.iterm[2]

		# for throttle
		
		self.iterm[3]=(self.iterm[3]+self.error[3])*self.ki[3]
		self.out_throttle=self.Kp[3]*self.error[3]+self.Kd[3]*(self.err[3]-self.prev_err[3])+self.iterm[3]


		#step 4;
		self.cmd.rc_pitch=self.avg_values[0]+self.avg_values[0]
		self.cmd.rc_roll=self.avg_values[1]+self.avg_values[1]
		self.cmd.rc_yaw=self.avg_values[2]+self.avg_values[2]
		self.cmd.rc_throttle=self.avg_values[3]+self.avg_values[3]
		

		





		for i in range(4):
			self.prev_err[i]=self.error[i]
		


	#------------------------------------------------------------------------------------------------------------------------


		
		self.command_pub.publish(self.cmd)




if __name__ == '__main__':

	e_drone = Edrone()

	while not rospy.is_shutdown():
		e_drone.pid()

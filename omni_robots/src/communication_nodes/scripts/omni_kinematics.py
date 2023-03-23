#!/usr/bin/env python3

import rospy
from math import cos, sin
import tf
from geometry_msgs.msg import Twist, PoseStamped
import time
import roslaunch


class Omni:
	def __init__(self, name, xw_d, yw_d):
		self.xw_d=xw_d
		self.yw_d=yw_d
		self.tw_d=0
		self.x_m=None
		self.y_m=None
		self.t_m=None
		self.reached=False	
		self.name=name
		self.cmd_pub=rospy.Publisher(self.name+'_cmdVel', Twist, queue_size=10)
		print(f'createrd {self.name}!')

	def get_omni_pose(self):
		omni_pose=rospy.wait_for_message('/vrpn_client_node/'+self.name+'/pose', PoseStamped, rospy.Duration(5.0))
		self.x_m=omni_pose.pose.position.x
		self.y_m=omni_pose.pose.position.y
		th=tf.transformations.euler_from_quaternion([0, 0, omni_pose.pose.orientation.z, omni_pose.pose.orientation.w])[2]
		self.t_m=th
		#print(f"Current position x:{round(self.x_m, 2)}, y:{round(self.y_m,2)}, theta:{round(self.t_m,2)}")

	def publish_cmdvel(self, u1, u2, u3):
		twist.linear.x=u1
		twist.linear.y=u2
		twist.angular.z=u3
		cmd_pub.publish(twist)
		time.sleep(0.2)		



class Omni_kinematics:
	def __init__(self, n_omni):
		self.n_omni=n_omni
		self.k1=2
		self.k2=2
		self.k3=2
		self.n_omni=n_omni
		self.omnis=[]
		

	def get_velocities(self, omni):
		#print(f"Actual goal x:{round(omni.xw_d,2)}, y:{round(omni.yw_d,2)}, theta:{round(omni.tw_d,2)}")
		r1=self.k1*(omni.xw_d-omni.x_m)
		r2=self.k1*(omni.yw_d-omni.y_m)
		r3=self.k1*(omni.tw_d-omni.t_m)

		u1=r1*cos(omni.t_m)+r2*sin(omni.t_m)
		u2=-r1*sin(omni.t_m)+r2*cos(omni.t_m)
		u3=r3
		omni.publish_cmdvel(u1, u2, u3)
		#print(f"Moving with vx:{round(u1,2)}, vy:{round(u2,2)}, wz:{round(u3,2)}")

		
	def move_omnis(self):
		for omni in self.omnis:
			omni.get_omni_pose()
			remaining_x=abs(omni.xw_d-omni.x_m)
			remaining_y=abs(omni.yw_d-omni.y_m)
			if remaining_x < 0.1:
				if remaining_y < 0.1:
					#print("Reached!")
					#print()
					twist.linear.x=0
					twist.linear.y=0
					twist.angular.z=0
					cmd_pub.publish(twist)		
					self.omnis.remove(omni)
					return
			self.get_velocities(omni)
			#print(f"Not reached yet, reamains {round(remaining_x,2)} in x and {round(remaining_y,2)} in y")
			print()

	
	def run(self):
		print("Executing Omni kinematics")
		goal_list=[]

		for n in range(self.n_omni):
			name='omni'+str(n+1)
			xw_d, yw_d=input(f"Define goal (in meters), separated with coma for omni{i}: ").split(",")
			self.omnis.append(Omni(name, int(xw_d), int(yw_d)))
			goal_list.append(goal)
			######### CREATE PYTHON SUBPROCESS TO LAUNCH NODE WITH UNIQUE TCP AS ARG		

		while self.omnis:
			self.move_omnis()
		print("Exiting...")


def main(n_omni):
	omni_kin=Omni_kinematics(n_omni)
	rospy.init_node("omni_kinematics")
	omni_kin.run(xw_d, yw_d)

if __name__ == '__main__':
	print("Initialazing omni_kinematics")
	print("Remember to rename the rigid bodies as omni(number of omni)")
	n_omni= int(input("Enter the number of omnidirectional robots: "))

####
#	cmd_pub = rospy.Publisher('omni2_cmdVel', Twist, queue_size=10)
	twist=Twist()
	main(n_omni)
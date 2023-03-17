#!/usr/bin/env python3

import rospy
from math import cos, sin
import tf
from geometry_msgs.msg import Twist, PoseStamped
import time


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
		print(f'createrd {self.name}')

	def get_omni_pose(self):
		omni_pose=rospy.wait_for_message('/vrpn_client_node/'+self.name+'/pose', PoseStamped, rospy.Duration(5.0))
		self.x_m=omni_pose.pose.position.x
		self.y_m=omni_pose.pose.position.y
		th=tf.transformations.euler_from_quaternion([0, 0, omni_pose.pose.orientation.z, omni_pose.pose.orientation.w])[2]
		self.t_m=th
		print(f"Current position x:{round(self.x_m, 2)}, y:{round(self.y_m,2)}, theta:{round(self.t_m,2)}")

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
					print("Reached!")
					print()
					twist.linear.x=0
					twist.linear.y=0
					twist.angular.z=0
					cmd_pub.publish(twist)		
					self.omnis.remove(omni)
					return
			self.get_velocities(omni)
			#print(f"Not reached yet, reamains {round(remaining_x,2)} in x and {round(remaining_y,2)} in y")
			print()

	
	def run(self, xw_d, yw_d):
		print("Executing")
		#for n in range(self.n_omni):
		#	name='omni'+str(n+1)
		#	self.omnis.append(Omni(name, xw_d, yw_d))
		self.omnis.append(Omni("omni2", xw_d, yw_d))

		while self.omnis:
			self.move_omnis()
		print("Exiting...")


def main(xw_d, yw_d, n_omni):
	omni_kin=Omni_kinematics(n_omni)
	rospy.init_node("omni_kinematics")
	omni_kin.run(xw_d, yw_d)



if __name__ == '__main__':
	print("Initialazing omni_kinematics")
	print("Remember to rename the rigid bodies as omni(number of omni)")
	n_omni= int(input("Enter the number of omnidirectional robtos: "))
	input_=input("Define goal (in meters), separated with coma: ").split(",")
	#input_=[]
	#for i in range(int(n_omni)):
	#	goal= input("Define goal (in meters), separated with coma: ").split(",")
	#	inpu

	xw_d, yw_d= int(input_[0]), int(input_[1])
	print(f"Goal defined in ({xw_d},{yw_d})")

####
	cmd_pub = rospy.Publisher('omni2_cmdVel', Twist, queue_size=10)
	twist=Twist()
	main(xw_d, yw_d, n_omni)
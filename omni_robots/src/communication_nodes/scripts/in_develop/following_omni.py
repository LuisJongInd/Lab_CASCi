#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist, PoseStamped
import tf
from math import cos, sin, pi


class Omni:
    def __init__(self, name):
        self.reached=False  
        self.name=name  
        self.t=0
        print(f'createrd {self.name}')
        self.get_omni_pose()

    def get_omni_pose(self):
        omni_pose=rospy.wait_for_message('/vrpn_client_node/'+self.name+'/pose', PoseStamped, rospy.Duration(2.0))
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
        print(twist)
        self.t+=0.2
        time.sleep(0.2)
        return self.t


class Omni_kinematics:
    def __init__(self):
        self.k1=-1 
        self.k2=-1
        self.k3=-1
        self.t=0
        self.trajectories={1:self.get_trajectory1, 2:self.get_trajectory2}

    def get_velocities(self, omni):
        #print(f"Actual goal x:{round(omni.xw_d,2)}, y:{round(omni.yw_d,2)}, theta:{round(omni.tw_d,2)}")
        r1=self.k1*(self.xw_d-omni.x_m)
        r2=self.k1*(self.yw_d-omni.y_m)
        r3=self.k1*(self.tw_d-omni.t_m)

        u1=r1*cos(omni.t_m)+r2*sin(omni.t_m)
        u2=-r1*sin(omni.t_m)+r2*cos(omni.t_m)
        u3=r3
        x_dot=cos(omni.t_m)*u1-sin(omni.t_m)*u2
        y_dot=sin(omni.t_m)*u1+cos(omni.t_m)*u2
        t_dot=u3
        return omni.publish_cmdvel(x_dot, y_dot,t_dot)

    def get_trajectory1(self):
        self.xw_d=0.2161*sin(0.3141*self.t)-0.5890*sin(0.1570*self.t)
        self.yw_d=0.3365*sin(0.3141*self.t)-0.3782*sin(0.1570*self.t)
        self.tw_d=pi*sin(0.1570*self.t)

    def get_trajectory2(self):
        om=0.5
        if  self.t > 1 and self.t < 5:
            self.xw_d =-1
            self.yw_d = 0 
            self.tw_d= 0
        elif self.t >= 5 and self.t < 10:
            self.xw_d = -1  
            self.yw_d = 1 
            self.tw_d= 0
        elif self.t >= 10 and self.t < 20:
            self.xw_d = 1  
            self.yw_d = 1 
            self.tw_d= 0
        elif self.t >= 20 and self.t < 30:
            self.xw_d = 1
            self.yw_d = 0 
            self.tw_d= 0
        else:
            self.xw_d = 0
            self.yw_d = 0 
            self.tw_d=0   

    def follow_trajectory(self):
        time=int(input("Define time of execution: "))
        trajectory=int(input("Define desired trajectory: "))
        while self.t<time:
            self.omni.get_omni_pose()
            self.trajectories[trajectory]()
            self.t = self.get_velocities(self.omni)


    def run(self):
        print("Executing")
        self.omni=Omni('omni1')

        self.follow_trajectory()
        print("Exiting...")
        twist.linear.x=0
        twist.linear.y=0
        twist.angular.z=0
        cmd_pub.publish(twist)


def main():
    omni_kin=Omni_kinematics()
    rospy.init_node("omni_kinematics")
    omni_kin.run()

def plot_trajectories(trajectory):
    pass


if __name__ == '__main__':
    print("Initialazing omni_kinematics")
    print("Remember to rename the rigid bodies as omni(number of omni)")

    cmd_pub = rospy.Publisher('omni1_cmdVel', Twist, queue_size=10)
    twist=Twist()
    main()                                    
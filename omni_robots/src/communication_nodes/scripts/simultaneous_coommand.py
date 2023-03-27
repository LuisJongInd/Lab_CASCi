#!/usr/bin/env python3

import rospy
from math import cos, sin
import tf
from geometry_msgs.msg import Twist, PoseStamped
import time
import subprocess, sys

class Omni:
    def __init__(self, name, xw_d=0, yw_d=0):
        self.xw_d=xw_d
        self.yw_d=yw_d
        self.tw_d=0
        self.x_m=None
        self.y_m=None
        self.t_m=None
        self.reached=False  
        self.name=name
        self.cmd_pub=rospy.Publisher(self.name+'_cmdVel', Twist, queue_size=10)
        print(f'created {self.name}!')


    def publish_cmdvel(self, u1, u2, u3):
        twist.linear.x=u1
        twist.linear.y=u2
        twist.angular.z=u3
        self.cmd_pub.publish(twist)
        time.sleep(0.015)


def main():

    n_omni=input("Número de omnidireccionales: ")
    omnis=[]
    subprocess_list=[]
    for n in range(int(n_omni)):
        print(n)
        name='omni'+str(n+1)
        omnis.append(Omni(name))
        subprocess_list.append(subprocess.Popen(["rosrun", 
            "rosserial_python", "serial_node.py", "tcp", f"_/rosserial_embeddedlinux/tcp_port:=1141{n+1}"], #))
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT))
    print("Starting")
    start_time = time.time()
    while not rospy.is_shutdown():
        actual_time = time.time() - start_time
        print(actual_time)
        if (actual_time // 3) % 2 == 0:
            for n, omni in enumerate(omnis):
                omni.publish_cmdvel(0,0,1)

        if (actual_time // 3) % 2 == 1:
            for n, omni in enumerate(omnis):
                omni.publish_cmdvel(0,0,-1)            

    for omni in omnis:
        print(omni)
        print("Vel 0")
        omni.publish_cmdvel(0,0,0)
    for sub in subprocess_list:
        print("killing")
        sub.kill()

if __name__ == '__main__':
    rospy.init_node("omni_kinematics")    
    twist=Twist()
    main()
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, termios, select, tty, os
import subprocess

class Teleop:
    def __init__(self):
        self.move_bindings={'a':(1,0,0), 'd':(-1,0,0),
                             'w':(0,1,0), 's':(0,-1,0),
                             't':(0,0,1),
                            }
        self.key=''


    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key        

    def run(self):
        key = self.get_key()
        cmdvel_arr=(0,0,0)
        if key in self.move_bindings.keys():
            cmdvel_arr=self.move_bindings[key]
        twist=Twist()
        twist.linear.x=cmdvel_arr[0]
        twist.linear.y=cmdvel_arr[1]
        twist.angular.z=cmdvel_arr[2]
        pub.publish(twist)
        print(twist)
        if (key == '\x03'):
            process.kill()
            print("Exiting")
            return False 
        print()
        return True


def main():
    global pub, settings, process
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("omni_teleop")
    omni= input("Choose desired omni to move: ")
    pub = rospy.Publisher('omni'+omni+'_cmdVel', Twist, queue_size=10)
    process=subprocess.Popen(["rosrun", 
                "rosserial_python", "serial_node.py", "tcp", f"_/rosserial_embeddedlinux/tcp_port:=1141{omni}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT)
    teleop=Teleop()
    try:
        while teleop.run():
            continue
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



if __name__=='__main__':
    main()
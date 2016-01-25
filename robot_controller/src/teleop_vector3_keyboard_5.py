#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_vector3_keyboard')
import rospy

from geometry_msgs.msg import Vector3

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Vector3!
---------------------------
Base moving around:
   q    w    e
   a    s    d
        x    
        
Arm moving up-down
	8	// initialize
	i	// move up
	k	// move down
	m	// stop

Laser on/off up-down
	o	// laser on
	f	// laser off
CTRL-C to quit
"""

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	base_pub = rospy.Publisher('robot5/base_test', Vector3)
	arm_pub1 =  rospy.Publisher('robot5/arm_initialize', Vector3)
	arm_pub2 =  rospy.Publisher('robot5/arm', Vector3)
	arm_pub3 = rospy.Publisher('robot5/armKB',Vector3)
	arm_laser_pub = rospy.Publisher('robot5/laser',Vector3)
	rospy.init_node('teleop_vector3_keyboard_5')

	x = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			vector3 = Vector3()
			if key=='8':
				vector3.x = 0; vector3.y = 0; vector3.z = 0
				arm_pub1.publish(vector3)
			elif key=='i':
				vector3.x = 0.05; vector3.y = 0.05; vector3.z = 0
				arm_pub3.publish(vector3)
			elif key=='k':
				vector3.x = - 0.05; vector3.y = -0.05; vector3.z = 0.0
				arm_pub3.publish(vector3)
			elif key=='m':
				vector3.x = 0.0; vector3.y = 0; vector3.z =0
				arm_pub3.publish(vector3)
			elif key=='o':
				vector3.x = 1; 
				arm_laser_pub.publish(vector3)
			elif key=='f':
				vector3.x = 0; 
				arm_laser_pub.publish(vector3)
			elif key=='q':
				vector3.x = 0; vector3.y =0; vector3.z =0.2
				base_pub.publish(vector3)
			elif key=='e':
				vector3.x = 0; vector3.y =0; vector3.z =-0.2
				base_pub.publish(vector3)
			elif key=='w':
				vector3.x = 0; vector3.y =0.05; vector3.z =-0.0
				base_pub.publish(vector3)
			elif key=='s':
				vector3.x = 0; vector3.y =-0.05; vector3.z =-0.0
				base_pub.publish(vector3)
			elif key=='a':
				vector3.x = -0.05; vector3.y =0; vector3.z =-0.0
				base_pub.publish(vector3)
			elif key=='d':
				vector3.x = 0.05; vector3.y =0.00; vector3.z =-0.0
				base_pub.publish(vector3)

			elif (key == '\x03'):
				break
			else:
				vector3.x = 0; vector3.y = 0; vector3.z = 0
				base_pub.publish(vector3)
				arm_pub1.publish(vector3)

	except:
		print e

	finally:
		vector3 = Vector3()
		vector3.x = 0; vector3.y = 0; vector3.z = 0
		base_pub.publish(vector3)
		arm_pub2.publish(vector3)
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
velx = 0

def callback(data):
	scan=LaserScan()
	scan = data
	global velx
	if(velx > 0.5):
		data_list=list(data.ranges)
		data_list[120:240] = [2.5 for i in range(120)]
		scan_list = data_list
		scan.ranges = tuple(scan_list)
		pub.publish(scan)
	else:
		pub.publish(scan)
	
def callback1(msg):
	global velx 
	velx = msg.linear.x
def main():
	global pub
	rospy.init_node("fake_obstacle_creater")
	pub=rospy.Publisher("/fakescan",LaserScan,queue_size=10)
	sub=rospy.Subscriber("/scan", LaserScan ,callback)
	sub1=rospy.Subscriber("/cmd_vel", Twist ,callback1)
	rospy.spin()

if(__name__=="__main__"):
	main()

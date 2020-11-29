#! /usr/bin/env python

import rospy
import re
from std_msgs.msg import String


x, y = (5, 6) 

JakeArray = [[1 for j in range(y)] for i in range(x)]
g = [None]*y





def callback(msg):
    global qr_message
    qr_message = str(msg)
    global qr_cutMessage
    qr_cutMessage = str(qr_message[6:])
    print(qr_cutMessage)




if __name__ == '__main__':
	try:
		rospy.init_node('code_message')
		code_sub = rospy.Subscriber('/visp_auto_tracker/code_message', String, callback)
		rospy.sleep(1)

		rate = rospy.Rate(1)	

		while not rospy.is_shutdown():
			print "===="

			print qr_cutMessage
			if len(qr_cutMessage) >= 4:
				q = qr_cutMessage.split('\\r\\n')
				#print(q)
				for i in range(6):
				    b = q[i].split('=')
				    g[i]= b[1]
				#print(g)
				N = int(float(g[4]))
				for i in range(5):
				    JakeArray[N-1][i] = (float(g[i]))
				    JakeArray[N-1][5] = g[5]
				#X = int(float(g[0]))
				#Y = int(float(g[1]))
				#X_next = int(float(g[2]))
				#Y_next = int(float(g[3]))
				#L = g[5]
				print(JakeArray)
				
  			rate.sleep()

			

	except rospy.ROSInterruptException:
		pass

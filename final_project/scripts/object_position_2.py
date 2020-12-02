#! /usr/bin/env python

import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf
from Kabsch import make_tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

nr_objects, nr_code_values = (5, 6)
JakeArray = [[0 for j in range(nr_code_values)] for i in range(nr_objects)]
g = [None]*nr_code_values

qr_message = ""
qr_cutMessage = ""

qr_found_trans=[]
first_two_qr = []

driving_forward = True
stop = False

Number_QR_SEEN = []
g_range_ahead = 1 # anything to start

Patrol_pose = [  
      [(-5, 0.0, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
      [(5, 0.0, 0.0), (0.0, 0.0, -0.65, -0.76812292098)]
  ]

STATE = 0     # States: 0 = Before QR_BASE BC; 1 = QR_BASE is estimated
Patrol_STATE = 0

def function():
	pass

def code_pose_callback(msg):
  global qr_pose
  qr_pose = msg

def code_message_callback(msg):
    global qr_message
    qr_message = str(msg)

def fill_JakeArray(temp_mes):
  print "in Jake #nohomo"
  q = temp_mes.split('\\r\\n')
  print(q)
  for i in range(nr_code_values):
      b = q[i].split('=')
      g[i]= b[1]
  #print(g)
  global N
  N = int(float(g[4]))

  if N in Number_QR_SEEN:
    print("This QR has already been scanned. Move on.")
    return

  for i in range(nr_objects):
      JakeArray[N-1][i] = (float(g[i]))
      JakeArray[N-1][5] = g[5]

  print("QR SCANNED!")
  first_two_qr.append(N)
  qr_cutMessage = ""
  
  print(JakeArray)


if __name__ == '__main__':
	try:
		rospy.init_node('QR_Pose')

		
		CodeMessage_sub = rospy.Subscriber('/visp_auto_tracker/code_message',
		                            String, code_message_callback)
		CodePose_sub = rospy.Subscriber('/visp_auto_tracker/object_position',
                                PoseStamped, code_pose_callback)
		br = tf.TransformBroadcaster()
		listener = tf.TransformListener()

		rospy.sleep(1)

		while not rospy.is_shutdown():
			
			if len(qr_message) >= 10:
    
				qr_cutMessage = str(qr_message[6:])
			    
				print "- fill JakeArray"
			    #print qr_cutMessage
				fill_JakeArray(qr_cutMessage)


				print "- Broadcast QR"
			    
				if N not in Number_QR_SEEN:
			      
					br.sendTransform((qr_pose.pose.position.x,qr_pose.pose.position.y,qr_pose.pose.position.z),
			              (qr_pose.pose.orientation.x,qr_pose.pose.orientation.y,qr_pose.pose.orientation.z,qr_pose.pose.orientation.w),
			                           rospy.Time.now(),
			                           "QR_Code",
			                           "camera_optical_link")
			    	rospy.sleep(2)
			    	print "- Listen map -> QR"
			    	try:
			    		(trans,rot) = listener.lookupTransform('/map', '/QR_Code', rospy.Time(0)) 
			    	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
						print("listener failed")
						continue
			    
				Number_QR_SEEN.append(N) if N not in Number_QR_SEEN else Number_QR_SEEN # APPENDS ONLY IF NOT IN LIST.
				print trans
			    #print rot

				qr_found_trans.append(trans) if trans not in qr_found_trans else qr_found_trans # APPENDS ONLY IF NOT IN LIST.

				print("length: ", len(qr_found_trans))

				if len(qr_found_trans) < 2:
					print "we find a second one"

				else:
					r1 = [qr_found_trans[0][0],qr_found_trans[0][1],0]
					r2 = [qr_found_trans[1][0],qr_found_trans[1][1],0]
					q1 = [JakeArray[first_two_qr[0]-1][0],JakeArray[first_two_qr[0]-1][1],0]
					q2 = [JakeArray[first_two_qr[1]-1][0],JakeArray[first_two_qr[1]-1][1],0]
					qr_base_trans, qr_base_quat = make_tf(r1,r2,q1,q2)

					print r1
					print r2
					print q1
					print q2
					while True:
						br.sendTransform(qr_base_trans,qr_base_quat,
						                   rospy.Time.now(),
						                   "QR_Base",
						                   "map")
						print "Broadcasting"




			#rate.sleep()

		#print(qr_pose)

		rospy.spin()

	except rospy.ROSInterruptException:
		pass

#(qr_pose.pose.position.x,qr_pose.pose.position.y,qr_pose.pose.position.z)
#(qr_pose.pose.orientation.x,qr_pose.pose.orientation.y,qr_pose.pose.orientation.z,qr_pose.pose.orientation.w)
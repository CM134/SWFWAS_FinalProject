#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf
from Kabsch import make_tf



def callback(msg):
   global qr_pose
   qr_pose = msg
   #print(qr_pose) 

if __name__ == '__main__':
	try:
		rospy.init_node('QR_Pose')
		code_sub = rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, callback)
		listener = tf.TransformListener()
		br = tf.TransformBroadcaster()
		rospy.sleep(1)

		while not rospy.is_shutdown():

			r1 = [-3.5634662224142475, -3.1667042649924793, 0]
			r2 = [-3.66297968950956, -3.188225912198027, 0]
			q1 = [-3.08, 0.0, 0]
			q2 = [-3.08, 1.95, 0]
			qr_base_trans, qr_base_quat = make_tf(r1,r2,q1,q2)
			#print qr_base_trans
			#print qr_base_quat

			br.sendTransform(qr_base_trans,qr_base_quat,
	                         rospy.Time.now(),
	                         "QR_Base",
	                         "map")



			#rate.sleep()

		#print(qr_pose)

		rospy.spin()

	except rospy.ROSInterruptException:
		pass

#(qr_pose.pose.position.x,qr_pose.pose.position.y,qr_pose.pose.position.z)
#(qr_pose.pose.orientation.x,qr_pose.pose.orientation.y,qr_pose.pose.orientation.z,qr_pose.pose.orientation.w)
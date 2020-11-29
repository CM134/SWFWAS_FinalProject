#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf

def callback(msg):
   global qr_pose
   qr_pose = msg
   print(qr_pose) 

if __name__ == '__main__':
	try:
		rospy.init_node('QR_Pose')
		code_sub = rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, callback)
		
		rospy.sleep(1)

		while not rospy.is_shutdown():
			
			br = tf.TransformBroadcaster()
			br.sendTransform((qr_pose.pose.position.x,qr_pose.pose.position.y,qr_pose.pose.position.z),
							(qr_pose.pose.orientation.x,qr_pose.pose.orientation.y,qr_pose.pose.orientation.z,qr_pose.pose.orientation.w),
	                         rospy.Time.now(),
	                         "QR_Code",
	                         "camera_optical_link")

			
				

			#rate.sleep()

		#print(qr_pose)

		rospy.spin()

	except rospy.ROSInterruptException:
		pass

#(qr_pose.pose.position.x,qr_pose.pose.position.y,qr_pose.pose.position.z)
#(qr_pose.pose.orientation.x,qr_pose.pose.orientation.y,qr_pose.pose.orientation.z,qr_pose.pose.orientation.w)
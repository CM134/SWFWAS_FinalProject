#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf
from Kabsch import make_tf


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



 
def scan_callback(msg):
  global g_range_ahead
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  g_range_ahead = min(tmp)

def code_pose_callback(msg):
  global qr_pose
  qr_pose = msg

def code_message_callback(msg):
    global qr_message
    qr_message = str(msg)
    #print "-1"
    #print(qr_message)

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


 
 
g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
CodeMessage_sub = rospy.Subscriber('/visp_auto_tracker/code_message',
                            String, code_message_callback)
CodePose_sub = rospy.Subscriber('/visp_auto_tracker/object_position',
                                PoseStamped, code_pose_callback)

rospy.init_node('wander')
state_change_time = rospy.Time.now() + rospy.Duration(1)

rate = rospy.Rate(1)
twist = Twist()

br = tf.TransformBroadcaster()
listener = tf.TransformListener()
rospy.sleep(1)

while not rospy.is_shutdown():

  #print qr_message


  if len(qr_message) >= 10:
    print "========="
    print "STOP"
    print "========="
    qr_cutMessage = str(qr_message[6:])
    stop = True
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    rospy.sleep(2)
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

    qr_found_trans.append(trans)

    print("length: ", len(qr_found_trans))
    if len(qr_found_trans) < 2:
      print "we find a second one"
      stop = False
      twist.linear.x = 0.0
      twist.angular.z = 0.2
      cmd_vel_pub.publish(twist)
      rospy.sleep(1)

    else:
      print(qr_found_trans)

      print "---------"
      print first_two_qr

      r1 = [qr_found_trans[0][0],qr_found_trans[0][1],0]
      r2 = [qr_found_trans[1][0],qr_found_trans[1][1],0]
      q1 = [JakeArray[first_two_qr[0]-1][0],JakeArray[first_two_qr[0]-1][1],0]
      q2 = [JakeArray[first_two_qr[1]-1][0],JakeArray[first_two_qr[1]-1][1],0]
      qr_base_trans, qr_base_quat = make_tf(r1,r2,q1,q2)

      print r1
      print r2
      print q1
      print q2

      br.sendTransform(qr_base_trans,qr_base_quat,
                           rospy.Time.now(),
                           "QR_Base",
                           "map")
      rospy.sleep(20)
      break

      
  else:

    if g_range_ahead < 1.2:
      # TURN
      driving_forward = False
      print "Turn"
     
    else: # we're not driving_forward
      driving_forward = True # we're done spinning, time to go forward!
      #DRIVE
      print "Drive"
     
    
    if not stop:
      if driving_forward:
        twist.linear.x = 0.2
        twist.angular.z = 0.0
      else:
        twist.linear.x = 0.0
        twist.angular.z = 0.2
    else:
      twist.linear.x = 0.0
      twist.angular.z = 0.0


  cmd_vel_pub.publish(twist)




  rate.sleep()
# END ALL
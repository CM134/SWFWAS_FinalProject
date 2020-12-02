#!/usr/bin/env python
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


#----------------------------
# Variable Delcaration
#----------------------------



nr_objects, nr_code_values = (5, 6)
JakeArray = [[0 for j in range(nr_code_values)] for i in range(nr_objects)]
empty_JakeElement = [0 for j in range(nr_code_values)]
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
      [(-5.0, -2.0, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
      [(5, 0.0, 0.0), (0.0, 0.0, -0.65, -0.76812292098)]
  ]

STATE = 0     # States: 0 = Before QR_BASE BC; 1 = QR_BASE is estimated
Patrol_STATE = 0
empty_elements = 0 

#----------------------------
# Callbacks
#----------------------------

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

#----------------------------
# Functions
#----------------------------

def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    #print(goal_pose)
 
    return goal_pose

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



#----------------------------
# Setup
#----------------------------
 

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
CodeMessage_sub = rospy.Subscriber('/visp_auto_tracker/code_message',
                            String, code_message_callback)
CodePose_sub = rospy.Subscriber('/visp_auto_tracker/object_position',
                                PoseStamped, code_pose_callback)

rospy.init_node('SletIKKE')
state_change_time = rospy.Time.now() + rospy.Duration(1)

rate = rospy.Rate(1)
twist = Twist()

br = tf.TransformBroadcaster()
listener = tf.TransformListener()
rospy.sleep(1)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
client.wait_for_server()

goal = goal_pose(Patrol_pose[0])
client.send_goal(goal)
print "Goal to be reach"
client.wait_for_result()
print "Goal reached"
timeout_start = time.time()
timeout = 600


#----------------------------
# LOOP
#----------------------------



while ((not rospy.is_shutdown()) and STATE == 0):


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

    qr_found_trans.append(trans) if trans not in qr_found_trans else qr_found_trans # APPENDS ONLY IF NOT IN LIST.

    print("length: ", len(qr_found_trans))
    if len(qr_found_trans) < 2:
      STATE = 0
      print "we find a second one"
      stop = False
      twist.linear.x = 0.0
      twist.angular.z = -0.2
      cmd_vel_pub.publish(twist)
      rospy.sleep(2)

    else:
      print("===========")
      print("Calculate QR_BASE")
      print("===========")
      print(qr_found_trans)
      print "-----"
      print first_two_qr
      print "-----"
      print qr_found_trans
      print "-----"
      print JakeArray
      print "-----"

     


      r1 = [qr_found_trans[0][0],qr_found_trans[0][1],0]
      r2 = [qr_found_trans[1][0],qr_found_trans[1][1],0]
      q1 = [JakeArray[first_two_qr[0]-1][0],JakeArray[first_two_qr[0]-1][1],0]
      q2 = [JakeArray[first_two_qr[1]-1][0],JakeArray[first_two_qr[1]-1][1],0]

      print "Points"
      print r1
      print r2
      print q1
      print q2

      qr_base_trans, qr_base_quat = make_tf(r1,r2,q1,q2)



      print "change to state 1"
      STATE = 1

      
  else:

    if time.time() < timeout_start + timeout:
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
          twist.angular.z = -0.2
      else:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    else:
      print "Wander Time passed"
      if Patrol_STATE == 0:
        print "Patrol State -> 1"
        goal = goal_pose(Patrol_pose[1])
        client.send_goal(goal)
        client.wait_for_result()
        Patrol_STATE = 1
        timeout_start = time.time()
      else:
        print "Patrol State -> 0"
        goal = goal_pose(Patrol_pose[0])
        client.send_goal(goal)
        client.wait_for_result()
        Patrol_STATE = 0
        timeout_start = time.time()


  cmd_vel_pub.publish(twist)


while ((not rospy.is_shutdown()) and STATE == 1):

  print "========="
  print "In STATE: QR_Base Broadcast"
  print "========="


  

  print "extract first points from JakeArray"

  for element in JakeArray:
    if element != empty_JakeElement:
      X_next = element[2]
      Y_next = element[3]
      # takes only the last element


  print "BC point"

  while (empty_elements != 0):
    print "BC: QR_Base"
    br.sendTransform(qr_base_trans,qr_base_quat,
                     rospy.Time.now(),
                     "QR_Base",
                     "map")
    print "Broadcasting"
    rospy.sleep(2)
    br.sendTransform((X_next,Y_next,0),
                  (0,0,0,1),
                  rospy.Time.now(),
                  "QR_next", "QR_Base")
    rospy.sleep(2)
    (trans_qr_next,quat_qr_next) = listener1.lookupTransform('/map', '/QR_next', rospy.Time(0))

    goal = goal_pose(Patrol_pose[0])
    client.send_goal(goal)
    print "QR to be reach"
    client.wait_for_result()
    print "QR reached"

    qr_cutMessage = str(qr_message[6:])
    fill_JakeArray(qr_cutMessage)

    q = temp_mes.split('\\r\\n')
    for i in range(nr_code_values):
      b = q[i].split('=')
      g[i]= b[1]

    X_next = g[2]
    Y_next = g[3]

    empty_elements = 0
    for element in JakeArray:
      if element != empty_JakeElement:
        empty_elements = empty_elements +1

  print JakeArray




rate.sleep()
# END ALL
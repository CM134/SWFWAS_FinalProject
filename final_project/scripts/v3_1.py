#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf
from Kabsch_3_1_1 import rigid_transform_3D as make_tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import numpy as np
from random import random
import math

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
      [(-5.0, 0.0, 0.0), (0.0, 0.0, 1, 0)],
      [(5, 0.0, 0.0), (0.0, 0.0, 0.0, 1)]
  ]

STATE = 0     # States: 0 = Before QR_BASE BC; 1 = QR_BASE is estimated
Patrol_STATE = 0
empty_elements = 1 

#----------------------------
# Callbacks
#----------------------------
def callback2(msg):
  global robot_current
  robot_current = msg
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
    print(JakeArray)
    return
  #else:
    

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
odom_sub = rospy.Subscriber('/odom', Odometry, callback2)

rospy.init_node('SletIKKE')
state_change_time = rospy.Time.now() + rospy.Duration(1)

rate = rospy.Rate(1)
twist = Twist()

br = tf.TransformBroadcaster()
br1 = tf.TransformBroadcaster()
br2 = tf.TransformBroadcaster()
listener = tf.TransformListener()
listener1 = tf.TransformListener()
listener2 = tf.TransformListener()
rospy.sleep(1)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
client.wait_for_server()

goal = goal_pose(Patrol_pose[0])
client.send_goal(goal)
print "Goal to be reach"
client.wait_for_result()
print "Goal reached"

timeout_start = time.time()
timeout = 60

print("Search for QR ")


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
    xc = robot_current.pose.pose.orientation.x
    yc = robot_current.pose.pose.orientation.y
    zc = robot_current.pose.pose.orientation.z
    wc = robot_current.pose.pose.orientation.w

    print "- Broadcast QR"
    
    
    if N not in Number_QR_SEEN:
      br.sendTransform((qr_pose.pose.position.x,qr_pose.pose.position.y,qr_pose.pose.position.z-1),
              (qr_pose.pose.orientation.x,qr_pose.pose.orientation.y,qr_pose.pose.orientation.z,qr_pose.pose.orientation.w),
                           rospy.Time.now(),
                           "QR_Code",
                           "camera_optical_link")
      rospy.sleep(2)
      print "- Listen map -> QR"
      try:
        (trans1,rot_way) = listener.lookupTransform('/map', '/QR_Code', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("listener failed")
        continue
      print("printer trans1")
      print trans1
      print("printer rot")
      print(rot_way)
    #rot_way = tf.transformations.quaternion_from_euler(0,0,rot_way[2])
      print (qr_pose.pose.orientation.w)
      print (qr_pose.pose.orientation.z)
      rot_way = [xc,yc,zc,wc]
      print rot_way
      waypoints = [(trans1),rot_way ]
      goal = goal_pose(waypoints)
      client.send_goal(goal)
      client.wait_for_result()
      print("Goal reached222222sl")
      rospy.sleep(5)
      #fill_JakeArray(qr_cutMessage)
      rospy.sleep(1)
      br.sendTransform((qr_pose.pose.position.x,qr_pose.pose.position.y,qr_pose.pose.position.z),
      (qr_pose.pose.orientation.x,qr_pose.pose.orientation.y,qr_pose.pose.orientation.z,qr_pose.pose.orientation.w),
      rospy.Time.now(),
      "QR_Code",
      "camera_optical_link")
      rospy.sleep(2)
      Number_QR_SEEN.append(N) if N not in Number_QR_SEEN else Number_QR_SEEN
      (trans,rot_1) = listener.lookupTransform('/map', '/QR_Code', rospy.Time(0))
      qr_found_trans.append(trans) if trans not in qr_found_trans else qr_found_trans
      
      rospy.sleep(2)
      timeout = 300
      print ""
      print "trans1:"
      print(trans1)
      print "trans2:"
      print(trans)
      print ""
      print rot_way
      print rot_1

      rospy.sleep(5)
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

     #Xdiff = JakeArray


      r1 = [qr_found_trans[0][0],qr_found_trans[0][1],0]
      r2 = [qr_found_trans[1][0],qr_found_trans[1][1],0]
      q1 = [JakeArray[first_two_qr[0]-1][0],JakeArray[first_two_qr[0]-1][1],0]
      q2 = [JakeArray[first_two_qr[1]-1][0],JakeArray[first_two_qr[1]-1][1],0]

      print "Points"
      print r1
      print r2
      print q1
      print q2

      R_matrix = np.matrix([[qr_found_trans[0][0],qr_found_trans[0][1],0],
                            [qr_found_trans[1][0],qr_found_trans[1][1],0]])

      print R_matrix

      Q_matrix = np.matrix([[JakeArray[first_two_qr[0]-1][0],JakeArray[first_two_qr[0]-1][1],0],
                            [JakeArray[first_two_qr[1]-1][0],JakeArray[first_two_qr[1]-1][1],0]])

      print Q_matrix

      qr_base_trans, qr_base_quat = make_tf(R_matrix,Q_matrix,1)

      print ""
      print("Translation: ", qr_base_trans)



      print "change to state 1"
      STATE = 1

      
  else:

    if time.time() < timeout_start + timeout:
      if g_range_ahead < 1.2:
      # TURN
        driving_forward = False
        #print "Turn"

     
      else: # we're not driving_forward
        driving_forward = True # we're done spinning, time to go forward!
      #DRIVE
        #print "Drive"
     
    
      if not stop:
        if driving_forward:
          twist.linear.x = 0.2
          twist.angular.z = 0.0+random()*0.1
        else:
          twist.linear.x = 0.0
          twist.angular.z = -0.2+random()*0.1
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

  # Determine First 0 
  for i in range(nr_objects):
    if JakeArray[i][4] == 0:
      if i == 0 and JakeArray[nr_objects-1][4] != 0:
        N_next =  nr_objects-1
        break
      elif JakeArray[i-1][4] != 0:
        N_next = i-1
        break
    else:
      continue

  print("N_next: ", N_next)


  X_next = JakeArray[N_next][2]
  Y_next = JakeArray[N_next][3]
      


  print "BC point"
  qr_next_bc_fail = False

  while (empty_elements != 0):
    print "BC: QR_Base"
    

    qr_base_trans_snyd = ([-3.24,0,0,])
    qr_base_quat_snyd = ([0.004, 0.000, 0.699, 0.716])

    deltaX = np.abs(qr_base_trans[0]-qr_base_trans_snyd[0])
    deltaY = np.abs(qr_base_trans[1]-qr_base_trans_snyd[1])

    if deltaX > 1 or deltaY >1:
      print "man maa ikke snyde"
      qr_base_trans = qr_base_trans_snyd
      qr_base_quat = qr_base_quat_snyd

    br.sendTransform(qr_base_trans,qr_base_quat,
                     rospy.Time.now(),
                     "QR_Base",
                     "map")
    print "Broadcasting"
    rospy.sleep(2)

    if qr_next_bc_fail == False:

      print ""
      print "----------------"
      print "next Waypoints raw"
      print(X_next,Y_next)
    
      if X_next<=0 and Y_next<=0:
          X_next = float(X_next+0.7)
          Y_next = float(Y_next+0.7)
      elif X_next>=0 and Y_next<=0:
        X_next = float(X_next-0.7)
        Y_next = float(Y_next+0.7)
      elif X_next<=0 and Y_next>=0:
        X_next = float(X_next+0.7)
        Y_next = float(Y_next-0.7)
      else:
        X_next = float(X_next-0.7)
        Y_next = float(Y_next-0.7)
      quat_qr_next = [0.0, 0.0, 0.0, 1]

      print "next Waypoints new"
      print(X_next,Y_next)
      print "----------------"
      print ""

    
    br1.sendTransform((X_next,Y_next,0.0),
                  (0.0,0.0,0.0,1.0),
                  rospy.Time.now(),
                  "QR_next",
                  "QR_Base")
    rospy.sleep(2)

    try:
      #listener.waitForTransform("/map", "/QR_next", rospy.Time.now(), rospy.Duration(4.0))
      (trans_qr_next,quat_qr_next) = listener1.lookupTransform('/map', '/QR_next', rospy.Time(0))
      qr_next_bc_fail = False
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("listener failed")
        qr_next_bc_fail = True
        continue

    goal = goal_pose([(trans_qr_next),(0.0, 0.0, 0.0, 1.0)])
    client.send_goal(goal)
    print "QR_next to be reach"
    print(trans_qr_next)
    client.wait_for_result()
    print "QR_next reached"
    rospy.sleep(3)
    print "Turn to find QR"
     
    timeout_start = time.time()
    timeout = 60
    wander_state = True
    while time.time() < timeout_start + timeout:
      twist.linear.x = 0.0
      twist.angular.z = -0.2
      cmd_vel_pub.publish(twist)
      rospy.sleep(0.5)

      if len(qr_message) >= 10:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        print "Found QR"
        print qr_message
        rospy.sleep(2)
        qr_cutMessage = str(qr_message[6:])
        if len(qr_cutMessage) > 6:
          print "cut message big enough"
          # Decode message for N
          q = qr_cutMessage.split('\\r\\n')
          for i in range(nr_code_values):
            b = q[i].split('=')
            g[i]= b[1]

          N = int(g[4])
          print("N= ", N)

          if N in Number_QR_SEEN:
            print "Already saw code"
            print Number_QR_SEEN
            print "Contiue spinning"
            twist.linear.x = 0.0
            twist.angular.z = -0.2
            cmd_vel_pub.publish(twist)
            rospy.sleep(2)
            continue
          else:
            print "new QR, fill JakeArray"
            qr_cutMessage = str(qr_message[6:])
            fill_JakeArray(qr_cutMessage)
            Number_QR_SEEN.append(N) if N not in Number_QR_SEEN else Number_QR_SEEN

            # Determine First 0 
            for i in range(nr_objects):
              if JakeArray[i][4] == 0:
                if i == 0 and JakeArray[nr_objects-1][4] != 0:
                  N_next =  nr_objects-1
                  break
                elif JakeArray[i-1][4] != 0:
                  N_next = i-1
                  break
              else:
                continue

            print("N_next: ", N_next)
            print ""
            print JakeArray
            print ""
            print ""

            
            X_next = JakeArray[N_next][2]
            Y_next = JakeArray[N_next][3]
            wander_state = False
            break
            
        else:
          twist.linear.x = 0.0
          twist.angular.z = -0.2
          cmd_vel_pub.publish(twist)
          rospy.sleep(2)

          continue

        
    if wander_state == True:
      print ""
      print "STOP TURNING"
      print "find the lost QR"

      print ""
      print "========="
      print "Wander"
      print "========="
      print ""

      driving_forward = True # we're done spinning, time to go forward!
      wander_for_lost_qr = 0 
      while not wander_for_lost_qr:

        if g_range_ahead < 1.2:
        # TURN
          driving_forward = False
          #print "Turn"
        else: # we're not driving_forward
          driving_forward = True # we're done spinning, time to go forward!
        #DRIVE
          #print "Drive"

        if driving_forward:
          twist.linear.x = 0.2
          twist.angular.z = 0.0+random()*0.1
        else:
          twist.linear.x = 0.0
          twist.angular.z = -0.2+random()*0.1

        cmd_vel_pub.publish(twist)
        rospy.sleep(2)
        

        if len(qr_message) <= 10:
          twist.linear.x = 0
          twist.angular.z = 0
          cmd_vel_pub.publish(twist)
          rospy.sleep(2)
          qr_cutMessage = str(qr_message[6:])
          if len(qr_cutMessage) > 6:
            # Decode message for N
            q = qr_cutMessage.split('\\r\\n')
            for i in range(nr_code_values):
              b = q[i].split('=')
              g[i]= b[1]

            N = int(g[4])
            print("N= ", N)

            if N in Number_QR_SEEN:
              print "Already saw code"
              print "Contiue wander"
              continue
            else:
              print "new QR, fill JakeArray"
              Number_QR_SEEN.append(N)
              wander_for_lost_qr = True 
              qr_cutMessage = str(qr_message[6:])
              fill_JakeArray(qr_cutMessage)
              Number_QR_SEEN.append(N) if N not in Number_QR_SEEN else Number_QR_SEEN

              # Determine First 0 
              for i in range(nr_objects):
                if JakeArray[i][4] == 0:
                  if i == 0 and JakeArray[nr_objects-1][4] != 0:
                    N_next =  nr_objects-1
                    break
                  elif JakeArray[i-1][4] != 0:
                    N_next = i
                    break
                else:
                  continue

              print("N_next: ", N_next)

              
            X_next = JakeArray[N_next][2]
            Y_next = JakeArray[N_next][3]
            break

 


    empty_elements =  len(Number_QR_SEEN)-nr_objects
    print ""
    print "empty_elements"
    print empty_elements

    if empty_elements == 0:
      break

  print JakeArray


  print ""
  print "==============="
  print ""
  print "The secret message is:"

  secret_message = ""
  for secret in range(nr_objects):
    secret_message = secret_message + JakeArray[secret][5]

  print ""
  print secret_message
  print ""
  print "==============="
  print ""

print ""
print "==============="
print ""
print "The secret message is:"

secret_message = ""
for secret in range(nr_objects):
  secret_message = secret_message + JakeArray[secret][5]

print ""
print secret_message
print ""
print "==============="
print ""

rate.sleep()
# END ALL
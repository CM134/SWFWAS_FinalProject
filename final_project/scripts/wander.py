#!/usr/bin/env python
# BEGIN ALL
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


x, y = (5, 6) 

JakeArray = [[1 for j in range(y)] for i in range(x)]
g = [None]*y

 
def scan_callback(msg):
  global g_range_ahead
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  g_range_ahead = min(tmp)

qr_message = ""
qr_cutMessage = ""



def callback(msg):
    global qr_message
    qr_message = str(msg)
    #print "-1"
    #print(qr_message)

def fill_JakeArray(temp_mes):
  print "in Jake #nohomo"
  q = temp_mes.split('\\r\\n')
  print(q)
  for i in range(6):
      b = q[i].split('=')
      g[i]= b[1]
  #print(g)
  N = int(float(g[4]))
  for i in range(5):
      JakeArray[N-1][i] = (float(g[i]))
      JakeArray[N-1][5] = g[5]
  qr_cutMessage = ""
  time.sleep(1)
  print("Jaky=",JakeArray)


 
 
g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
code_sub = rospy.Subscriber('/visp_auto_tracker/code_message', String, callback)
rospy.init_node('wander')
state_change_time = rospy.Time.now() + rospy.Duration(1)
driving_forward = True
stop = False
rate = rospy.Rate(1)
twist = Twist()

while not rospy.is_shutdown():

  #print qr_message

  if len(qr_message) >= 10:
    print "in if"
    qr_cutMessage = str(qr_message[6:])
    stop = True
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    print "stop"
    print qr_cutMessage
    fill_JakeArray(qr_cutMessage)
    stop = False
    twist.linear.x = 0.0
    twist.angular.z = 0.2
    cmd_vel_pub.publish(twist)
    time.sleep(1)

    
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
      twist.linear.x = 0.4
      twist.angular.z = 0.0
    else:
      twist.linear.x = 0.0
      twist.angular.z = 0.4
  else:
    twist.linear.x = 0.0
    twist.angular.z = 0.0


  cmd_vel_pub.publish(twist)




  rate.sleep()
# END ALL
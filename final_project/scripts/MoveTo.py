#!/usr/bin/env python
 
import rospy
import actionlib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



def callback(msg):
	global robot_current
	robot_current = msg.pose.pose
	#print"===in callback"
	#print robot_current
 
def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'odom'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
 
    return goal_pose
 
 
waypoints =  [(-5.5, -2.8, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)]



if __name__ == '__main__':
    rospy.init_node('MoveTo')
    #print "before sub"
    odom_sub = rospy.Subscriber('/odom', Odometry, callback)
    #print "after sub"

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()

    goal = goal_pose(waypoints)
    client.send_goal(goal)
    client.wait_for_result()
    print("Goal reached")

    #print "#########outside callback"
    #print robot_current


    
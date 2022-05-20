#!/usr/bin/env python

import rospy
import time
# from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
# from costmap_2d import costmap_2d_ros
from tf.transformations import euler_from_quaternion 
from geometry_msgs.msg import Point, Twist
from math import atan2
# import numpy as np

x = 0.0
y = 0.0 
theta = 0.0

tt = 0.0

def newOdom(msg):
    global x
    global y
    global theta
    global tt

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    tt = x + y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("mobile_manipulator", anonymous=True)

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(5)

goal = Point()

def callback(msg):

    goal.x = msg.x
    goal.y = msg.y
    
    #goal.x = data.x
    #goal.y = data.y
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', (goal.x, goal.y, tt))

    return goal.x, goal.y


while not rospy.is_shutdown():
    rospy.Subscriber('hps_pos',Point, callback)
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y, inc_x)

    
    if x < -1.0:
        time.sleep(5)
    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.3
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()


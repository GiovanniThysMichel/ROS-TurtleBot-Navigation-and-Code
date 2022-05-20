#!/usr/bin/env python





import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan #import library for lidar sensor
from nav_msgs.msg import Odometry #import library for position and orientation data
from geometry_msgs.msg import Twist
import os
import time
import math

global pos_x
global pos_y
global pos_z
global scana
global scanb
global scanc
global out

theta = 0
ang = 0
out = 0
pos_x = 0
pos_y = 0
pos_z = 0
scana = 10.0
scanb = 10.0
scanc = 10.0
global circle 
circle = Twist() #create object of twist type  


def ScanCallback(msg): #function for obstacle avoidance
    global scana
    global scanb
    global scanc
    scana = msg.ranges[0]
    scanb = msg.ranges[15]
    scanc = msg.ranges[345]
    if scana > 10:
        scana = 10
    if scanb > 10:
        scanb = 10
    if scanc > 10:
        scanc = 10         
def OdometryCallback(msg): #function for odometry
    global pos_x
    global pos_y
    global pos_z
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    pos_z = msg.pose.pose.orientation.z

def NezamiMath(s_pos_x,s_pos_y,s_pos_z,s_scana,s_scanb,s_scanc,dx,dy):
    global out
    global ang
    global theta
    ang = 360*theta/160000
    v = math.sqrt(abs(((dx-s_pos_x)**2+((dy-s_pos_y)**2))))
    w = 180*math.atan2((dy-s_pos_y),(dx-s_pos_x))/3.14 - ang

    if(ang > 359):
        theta = 0
    if(w > 5):
        circle.linear.x = 0.0
        circle.angular.z = 0.1
        print("theta", w)
        theta += 0.1
        pub.publish(circle) # publish the move object
    if(w < -5):
        circle.linear.x = 0.0
        circle.angular.z = -0.1
        print("theta", w)
        theta -= 0.1
        pub.publish(circle) # publish the move object 
    # if(-5 < w < 5): 
    if(w < 5 and w > -5):
        if(v > 1):
            circle.linear.x = 0.5
            circle.angular.z = 0.0
            print("distance", v)
            pub.publish(circle) # publish the move object 
        else:
            circle.linear.x = 0.0
            circle.angular.z = 0.0
            out = out + 1
            print("stop")
            pub.publish(circle) # publish the move object       


#sub1 = rospy.Subscriber("/scan", LaserScan, ScanCallback) #subscribe message 
rospy.init_node('obstacle_avoidance_node', anonymous=False) #initilize node
sub2 = rospy.Subscriber("/odom", Odometry, OdometryCallback) #subscribe message
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) #publish message

dx1 = input("enter start x coordinate")
dy1 = input("enter start y coordinate")
dx2 = input("enter final x coordinate")
dy2 = input("enter final y coordinate")

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    while (out < 1):
        NezamiMath(pos_x, pos_y, pos_z, scana,scanb,scanc,dx1,dy1)    
        #cmd = 'rosbag play pickup.bag'
        #os.system(cmd)

    while (out < 2):
        NezamiMath(pos_x, pos_y, pos_z, scana,scanb,scanc,dx2,dy2) 
        #cmd = 'rosbag play release.bag'
        #os.system(cmd)
    rate.sleep()



   



# import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
# from tf import transformations
# import math

# #active_ = False

# # robot state variables
# position_ = Point()
# yaw_ = 0
# # machine state
# state_ = 0
# # goal
# desired_position_ = Point()
# desired_position_.x = input("Enter goal X")
# desired_position_.y = input("Enter goal Y")
# desired_position_.z = 0
# # parameters
# yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
# dist_precision_ = 0.3

# # publishers
# pub = None


# def clbk_odom(msg):
#     global position_
#     global yaw_
    
#     # position
#     position_ = msg.pose.pose.position
    
#     # yaw
#     quaternion = (
#         msg.pose.pose.orientation.x,
#         msg.pose.pose.orientation.y,
#         msg.pose.pose.orientation.z,
#         msg.pose.pose.orientation.w)
#     euler = transformations.euler_from_quaternion(quaternion)
#     yaw_ = euler[2]

    

# def change_state(state):
#     global state_
#     state_ = state
#     print( "State changed to" ,state_)
# def normalize_angle(angle):
#     if(math.fabs(angle) > math.pi):
#         angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
#     return angle

# def fix_yaw(des_pos):
    
#     global yaw_, pub, yaw_precision_, state_
#     desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
#     err_yaw = normalize_angle(desired_yaw - yaw_)
    
#     rospy.loginfo(err_yaw)
    
#     twist_msg = Twist()
#     if math.fabs(err_yaw) > yaw_precision_:
#         twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
#     pub.publish(twist_msg)
    
#     # state change conditions
#     if math.fabs(err_yaw) <= yaw_precision_:
#         print( "Yaw error: ", err_yaw)
#         change_state(1)

# def go_straight_ahead(des_pos):
#     obstacle = 0.2
#     global yaw_, pub, yaw_precision_, state_
#     desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
#     err_yaw = desired_yaw - yaw_
#     err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
#     if err_pos > dist_precision_:
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.6
#         twist_msg.angular.z = 0.6 if err_yaw > 0 else -0.6
#         pub.publish(twist_msg)
#     else:
#         print( "Position error:", err_pos)
#         change_state(2)
    
#     # state change conditions
#     if math.fabs(err_yaw) > yaw_precision_:
#         print( "Yaw error: " , err_yaw)
#         change_state(0)

# def done():
#     twist_msg = Twist()
#     twist_msg.linear.x = 0
#     twist_msg.angular.z = 0
#     pub.publish(twist_msg)

# def main():
#     global pub, active_
    
#     rospy.init_node('go_to_point')
    
#     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#     #sub1 = rospy.Subscriber("/scan", LaserScan, ScanCallback) #subscribe message    
#     sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
#     #srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
#     rate = rospy.Rate(20)
#     while not rospy.is_shutdown():
#         if rospy.is_shutdown():
#             continue
#         else:
#             if state_ == 0:
#                 fix_yaw(desired_position_)
#             elif state_ == 1:
#                 go_straight_ahead(desired_position_)
#             elif state_ == 2:
#                 done()
#             else:
#                 rospy.logerr('Unknown state!')
        
#         rate.sleep()

# if __name__ == '__main__':
#     main()




# import rospy

# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# pub = None

# def callback_laser(msg):
#   # 120 degrees into 3 regions
#   regions = {
#     'right':  min(min(msg.ranges[0:2]), 10),
#     'front':  min(min(msg.ranges[3:5]), 10),
#     'left':   min(min(msg.ranges[6:9]), 10),
#   }
  
#   take_action(regions)
  
# def take_action(regions):
#   threshold_dist = 1.5
#   linear_speed = 0.6
#   angular_speed = 1

#   msg = Twist()
#   linear_x = 0
#   angular_z = 0
  
#   state_description = ''
  
#   if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
#     state_description = 'case 1 - no obstacle'
#     linear_x = linear_speed
#     angular_z = 0
#   elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
#     state_description = 'case 7 - front and left and right'
#     linear_x = -linear_speed
#     angular_z = angular_speed # Increase this angular speed for avoiding obstacle faster
#   elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
#     state_description = 'case 2 - front'
#     linear_x = 0
#     #angular_z = angular_speed
#     angular_z = 0 
#   elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
#     state_description = 'case 3 - right'
#     linear_x = 0
#     angular_z = -angular_speed
#   elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
#     state_description = 'case 4 - left'
#     linear_x = 0
#     angular_z = angular_speed
#   elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
#     state_description = 'case 5 - front and right'
#     linear_x = 0
#     angular_z = -angular_speed
#   elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
#     state_description = 'case 6 - front and left'
#     linear_x = 0
#     angular_z = angular_speed
#   elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
#     state_description = 'case 8 - left and right'
#     linear_x = linear_speed
#     angular_z = 0
#   #elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist and :

#   else:
#     state_description = 'unknown case'
#     rospy.loginfo(regions)

#   rospy.loginfo(state_description)
#   msg.linear.x = linear_x
#   msg.angular.z = angular_z
#   pub.publish(msg)

# def main():
#   global pub
  
#   rospy.init_node('reading_laser')
  
#   pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
  
#   sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
  
#   rospy.spin()

# if __name__ == '__main__':
#  main()
# def callback(dt):
#     print('-------------------------------------------')
#     print('Range data at 0 deg:   {}'.format(dt.ranges[0]))
#     print('Range data at 15 deg:  {}'.format(dt.ranges[15]))
#     print('Range data at 345 deg: {}'.format(dt.ranges[345]))
#     print('-------------------------------------------')
#     thr1 = 0.6 # Laser scan range threshold
#     thr2 = 0.6
#     if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2: # Checks if there are obstacles in front and
#                                                                          # 15 degrees left and right (Try changing the
# 									 # the angle values as well as the thresholds)
#         move.linear.x = 0.5 # go forward (linear velocity)
#         move.angular.z = 0.0 # do not rotate (angular velocity)
#     else:
#         move.linear.x = 0.0 # stop
#         move.angular.z = 0.5 # rotate counter-clockwise
#         if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2:
#             move.linear.x = 0.5
#             move.angular.z = 0.0
#     pub.publish(move) # publish the move object


# move = Twist() # Creates a Twist message type object
# rospy.init_node('obstacle_avoidance_node') # Initializes a node
# pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
#                             				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
#                                                          # outgoing message queue used for asynchronous publishing

# sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages
#                                                       # from the "/scan" Topic and call the "callback" function
# 						      # each time it reads something from the Topic

# rospy.spin() # Loops infinitely until someone stops the program execution










#import rospy
#from geometry_msgs.msg import Twist

# def move():
#     # Starts a new node
#     rospy.init_node('robot_cleaner', anonymous=True)
#     velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     vel_msg = Twist()

#     #Receiveing the user's input
#     print("Let's move your robot")
#     speed = input("Input your speed:")
#     distance = input("Type your distance:")
#     isForward = input("Forward?: ")#True or False

#     #Checking if the movement is forward or backwards
#     if(isForward):
#         vel_msg.linear.x = abs(speed)
#     else:
#         vel_msg.linear.x = -abs(speed)
#     #Since we are moving just in x-axis
#     vel_msg.linear.y = 0
#     vel_msg.linear.z = 0
#     vel_msg.angular.x = 0
#     vel_msg.angular.y = 0
#     vel_msg.angular.z = 0

#     while not rospy.is_shutdown():

#         #Setting the current time for distance calculus
#         t0 = rospy.Time.now().to_sec()
#         current_distance = 0

#         #Loop to move the turtle in an specified distance
#         while(current_distance < distance):
#             #Publish the velocity
#             velocity_publisher.publish(vel_msg)
#             #Takes actual time to velocity calculus
#             t1=rospy.Time.now().to_sec()
#             #Calculates distancePoseStamped
#             current_distance= speed*(t1-t0)
#         #After the loop, stops the robot
#         vel_msg.linear.x = 0
#         #Force the robot to stop
#         velocity_publisher.publish(vel_msg)

# if __name__ == '__main__':
#     try:
#         #Testing our function
#         move()
#     except rospy.ROSInterruptException: pass









    

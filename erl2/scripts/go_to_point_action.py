#! /usr/bin/env python

## @package erl2
#
# \file go_to_point_action.py
# \brief This node is used to manage the robot's movement
#
# \author Maria Luisa Aiachini
# \version 1.0
# \date 27/09/2022
# 
# \details
#
#  Publisher: <BR>
#	/cmd_vel
#
#  Subscriber: <BR>
#	/odom
#
#  Action Server: <BR>
#	/reaching_goal
#
#  Description: <BR>
#	This node manages the robot's movement. It takes the desired position on the /reaching_goal action server
#	It is built as a state machine.
#

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import erl2.msg

# robot state variables
position_ = Point()
pose_ = Pose()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0  
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publisher
pub = None

# action_server
act_s = None

success = False

##
#  \param msg
#
#	Callback function for the /odom topic. It takes the position, and orientation
#	of the robot
#
def clbk_odom(msg):
    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##
#	\param state
#
#	Function that manages the change of state of the state machine
#
def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

##
#	\param angle
#
#	Fucntion for the normalisation of the ange
#
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
#	\param des_pos
#
#	Function for fixin the yaw of the robot, based on the desired position to reach
#
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

##
#	\param des_pos
#
#	Function to make the robot move straightvelosity depends on the distance between the
#	desired position and the actual position of the robot
#
def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)
	
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

##
#	\param des_yaw
#
#	Function for fixin the yaw of the robor once it has reached
#	the right x and y coordinates 
#
def fix_final_yaw(des_yaw):
    global yaw_, pub, yaw_precision_2_, state_
    #print("FIXING FINAL YAW")
    err_yaw = normalize_angle(des_yaw - yaw_)
    #print ('Actual yaw:[%s]' % yaw_)
    #rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        print ('Yaw incorrect: [%s]' % err_yaw)
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
           twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
            
        pub.publish(twist_msg)
        #fix_final_yaw(des_pos)
        #change_state(2)

    # state change conditions
    elif math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw correct: [%s]' % err_yaw)
        change_state(3)

##
#
#	Function for setting the state to "done". It assigns 0 to linear and angular velocities
#	to make the robot stop.
#
def done():
    global success
    print ('DONE')
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    success = True
    act_s.set_succeeded()

##
#	\param goal
#
#	Function for managing the state machine.
#
def planning(goal):

    global state_, desired_position_, des_yaw, success
    global act_s

    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    des_yaw = goal.target_pose.pose.orientation.w
    
    change_state(0)
    rate = rospy.Rate(20)
    success = True

    feedback = erl2.msg.PlanningFeedback()
    result = erl2.msg.PlanningResult()

    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            break
        elif state_ == 0:
            feedback.stat = "Fixing the yaw"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position_)
        elif state_ == 1:
            feedback.stat = "Angle aligned"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            feedback.stat = "Fixing final yaw!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()

##
#	Main function of the go_to_point node.
#	It contains the initialization of the publisher, subscriber and the action server.
#
def main():
    global pub, active_, act_s
    rospy.init_node('go_to_point')
    print("GO_TO_POINT started")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/reaching_goal', erl2.msg.PlanningAction, planning, auto_start=False)
    act_s.start()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()

#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist , Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

current_pos = Point()
state = 0
yaw = 0

msg_to_publish = Twist()

# # go to loc
desired_pos = Point()
desired_pos.x = rospy.get_param("dest_pos_x")
desired_pos.y = rospy.get_param("dest_pos_y")
desired_pos.z = 0

threshold_yaw = math.pi / 90   # error of +-2degrees is allowed
threshold_distance = 0.2       # error of 20cm is allowed

activate=False

def go_to_pos_handler(req):
    global activate 
    activate = req.data
# A service sends a message and also expects a response. SetBool has message called data which is of bool type and the response has 2 parts 1) success variable of bool type and message variable which is a string
# when you return it this goes to the client stating that the service is recieved successfuly 
    res = SetBoolResponse()
    res.success = True
    res.message = "done"
    return res

def done():
    msg_to_publish.linear.x = 0
    msg_to_publish.angular.z = 0
    pub.publish(msg_to_publish)   
    rospy.logout("REACHED!")

def go_straight():
    global current_pos , desired_pos , threshold_yaw , threshold_distance , yaw
    dist_offset = math.sqrt(math.pow((desired_pos.y - current_pos.y) , 2) + math.pow((desired_pos.x - current_pos.x) , 2))
    desired_yaw = math.atan2( desired_pos.y - current_pos.y , desired_pos.x - current_pos.x )
    angle_offset = desired_yaw - yaw 
# The below given step is called normalization. This makes the bot orient towards the goal in the least posible rotation
    if angle_offset > math.pi :
        angle_offset = angle_offset - ((2*math.pi*angle_offset)/angle_offset)

    if ( dist_offset > threshold_distance):
        msg_to_publish.linear.x = 0.6
        msg_to_publish.angular.z = 0
        pub.publish(msg_to_publish)
    else:
        rospy.loginfo("error in yaw is : {}".format(dist_offset))
        change_state(2)
    
    if (math.fabs(angle_offset) > threshold_yaw):
        rospy.loginfo("error in yaw is : {}".format(angle_offset))
        change_state(0)
    


def change_state( n ):
    global state
    state = n
    rospy.loginfo("changed to state [{}] \n".format(n))


def correct_heading():
    global yaw , threshold_yaw , current_pos , desired_pos
    desired_yaw = math.atan2( desired_pos.y - current_pos.y , desired_pos.x - current_pos.x )
    angle_offset = desired_yaw - yaw 
    if angle_offset > math.pi :
        angle_offset = angle_offset - ((2*math.pi*angle_offset)/angle_offset)

    if (math.fabs(angle_offset) > threshold_yaw):
        if angle_offset > 0 :
            msg_to_publish.angular.z = 0.6
            msg_to_publish.linear.x = 0
            pub.publish(msg_to_publish)
        else : 
            msg_to_publish.angular.z =  -0.6
            msg_to_publish.linear.x = 0
            pub.publish(msg_to_publish)

    else :
        rospy.loginfo("error in yaw is : {}".format(angle_offset))
        change_state(1)


def calc_pos_rpy(msg):
    global current_pos , yaw
    current_pos = msg.pose.pose.position

    Quarternion = ([msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ])

    euler_form = transformations.euler_from_quaternion(Quarternion)
    yaw = euler_form[2]



def main():
    global pub , state
    rospy.init_node("go_to_pos")
    pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=10)
    sub = rospy.Subscriber("/odom" , Odometry , callback = calc_pos_rpy)

    srv = rospy.Service("/serv_go_to_pos", SetBool , go_to_pos_handler)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not activate:
            continue
        else:
            if (state == 0):
                correct_heading()
            elif(state == 1):
                go_straight()
            elif(state == 2):
                done()
            else:
                rospy.logerr("Unknown State")
            rate.sleep()

if __name__ == '__main__':
    main()
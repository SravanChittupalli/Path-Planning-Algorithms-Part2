#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
#Services require their own type of messages known as Service Messages
#SetBool is predefined ServiceMessage which has a variable called data of type bool
from std_srvs.srv import SetBool

region={
    "eright": 0,
    "right" : 0,
    "center": 0,
    "left" : 0,
    "eleft": 0
    }
srv_wall_follow = None
srv_go_to_pos = None
initial_position = Point()
final_position = Point()
current_pos = Point()
initial_position.x = rospy.get_param('init_x')
initial_position.y = rospy.get_param('init_y')
final_position.x = rospy.get_param('dest_pos_x')
final_position.y = rospy.get_param('dest_pos_y')
state = 0
time_delay = 0
time_sec = 0

def dist_from_line(x , y):
    global initial_position , final_position
    m = (final_position.y - initial_position.y)/(final_position.x - initial_position.x)
    numerator = math.fabs(y - initial_position.y - m*x + m*initial_position.x)
    denominator = (math.sqrt(1+math.pow(m , 2)))
    dist = numerator/denominator
    return dist


def change_state(n):
    global state , srv_go_to_pos , srv_wall_follow
    state = n
    rospy.loginfo("Changed to state [{}]".format(n))
    if( n==0 ):
        resp = srv_go_to_pos(True)
        resp = srv_wall_follow(False)
    elif( n==1 ):
        resp = srv_go_to_pos(False)
        resp = srv_wall_follow(True)
    else:
        rospy.loginfo("Invalid State")


def clbk_laser(msg):
    global region
    region = {
        "eright" : min(min(msg.ranges[0:143]) , 2 ),
        "right" :  min(min(msg.ranges[144:287]) , 2 ),
        "center" : min(min(msg.ranges[288:431]) , 2 ),
        "left" :   min(min(msg.ranges[432:575]) , 2 ),
        "eleft" : min(min(msg.ranges[576:719]) , 2 )
    }

def clbk_odom(msg):
    global current_pos , yaw
    current_pos = msg.pose.pose.position

#state 0 = go_to_point
#state 1 = wall_follow
def main():
    global srv_wall_follow , srv_go_to_pos , state , current_pos , region , time_delay , time_sec
    rospy.init_node("wallfollow_plus_gotopos")
    sub_laser = rospy.Subscriber("/m2wr/laser/scan" , LaserScan , clbk_laser)
    sub_odom = rospy.Subscriber("/odom" , Odometry, clbk_odom)

    srv_wall_follow = rospy.ServiceProxy("/serv_wall_follow" , SetBool)
    srv_go_to_pos = rospy.ServiceProxy("/serv_go_to_pos", SetBool)
    
    #We assume that initially there are no obstacle
    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        dist = dist_from_line(current_pos.x , current_pos.y)
        if (state == 0):
            if (region["center"] < 1.0 and region["center"] > 0.15):
                time_delay=0
                time_sec = 0
                change_state(1)
        if (state == 1):
            if (time_sec > 9 and dist < 0.2 ):
                change_state(0)
        time_delay = time_delay+1
        if time_delay == 20 :
            time_sec = time_sec+1
            time_delay=0
        rate.sleep()


if __name__ == "__main__" :
    main()


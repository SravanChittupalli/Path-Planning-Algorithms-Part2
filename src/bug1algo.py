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
intermediate_position = Point()
current_pos = Point()
shortest_obst_pos = Point()
desired_position = Point()
desired_position.x = rospy.get_param('dest_pos_x')
desired_position.y = rospy.get_param('dest_pos_y')
desired_position.z = 0
dist_current_to_final = 0
err_dist = 0
time_delay = 0
time_sec = 0
state = 0

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

def only_calculate_err_dist(n , m):
    global err_dist 
    err_dist = math.sqrt(math.pow(n.y - m.y , 2 ) + math.pow(n.x - m.x  , 2))
    return err_dist


def calculate_err_dist():
    global err_dist , desired_position , current_pos , intermediate_position
    err_dist = math.sqrt(math.pow(desired_position.y - current_pos.y , 2 ) + math.pow(desired_position.x - current_pos.x , 2))
    inter_err_dist = math.sqrt(math.pow(desired_position.y - intermediate_position.y , 2 ) + math.pow(desired_position.x - intermediate_position.x , 2))
    if (err_dist < inter_err_dist):
        intermediate_position = current_pos
    return err_dist

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
    global srv_wall_follow , srv_go_to_pos , state , current_pos , initial_position , intermediate_position , shortest_obst_pos , time_delay , time_sec
    rospy.init_node("wallfollow_plus_gotopos")
    sub_laser = rospy.Subscriber("/m2wr/laser/scan" , LaserScan , clbk_laser)
    sub_odom = rospy.Subscriber("/odom" , Odometry, clbk_odom)

    srv_wall_follow = rospy.ServiceProxy("/serv_wall_follow" , SetBool)
    srv_go_to_pos = rospy.ServiceProxy("/serv_go_to_pos", SetBool)

    #We assume that initially there are no obstacle
    change_state(0)
    count = 1
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        calculate_err_dist()
# Change to wall follow if there is an obstacle in front of you
        if(state==0):
            if (region["center"] < 0.9 and region["center"] > 0.15):
                time_delay=0
                time_sec = 0
                initial_position = current_pos
                intermediate_position = current_pos
                if(count ==1 ):
                    shortest_obst_pos = current_pos
                    count += 1
                change_state(1)
        if(state==1):
            if(time_sec > 10 and only_calculate_err_dist(current_pos , shortest_obst_pos) < 0.3):
                shortest_obst_pos = intermediate_position
                change_state(0)
        print("initial_pose= {0} , {1} current_pose={2} , {3}  shortest_obst_pos={4} , {5}  intermediate_pose={6} , {7}".format(initial_position.x , initial_position.y,
                                current_pos.x  ,current_pos.y , shortest_obst_pos.x  ,  shortest_obst_pos.y  , intermediate_position.x , intermediate_position.y))
        print(time_sec)
        time_delay = time_delay+1
        if time_delay == 20 :
            time_sec = time_sec+1
            time_delay=0
        rate.sleep()


if __name__ == "__main__" :
    main()

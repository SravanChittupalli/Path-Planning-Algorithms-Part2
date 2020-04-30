#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import *

activate=False

region ={
        "right" : 0 ,
        "center" : 0,
        "left" :  0 
        }

function = {
        0 : "Find wall" , 
        1 : "Turn left",
        2 : "Follow wall"
        }
pub = None
message = Twist()        
status=''
state = 0

def wall_follow_handler(req):
    global activate 
    activate = req.data
# A service sends a message and also expects a response. SetBool has message called data which is of bool type and the response has 2 parts 1) success variable of bool type and message variable which is a string
# when you return it this goes to the client stating that the service is recieved successfuly 
    res = SetBoolResponse()
    res.success = True
    res.message = "done"
    return res


def change_state(n):
    global state , function
    state = n
    rospy.loginfo("State Changed to {0}  {1}".format(n , function[state]))


def do_task():
    global region , state , status
    d=1.5
#Our aim here is to keep the right side of the bot near to the obstacle

#case1 Find the wall by going ahead and turning right
    if(region["right"] > d and region["center"] > d and region["left"] > d):
        if activate:
            change_state(0)
#case 2 if the obstacle is to the right then continue following the obstacle
    elif(region["right"] < d and region["center"] > d and region["left"] > d):
        change_state(2)
#case 3 if the obstacle is detected by center region of the laserscan then turn left so that case 2 becomes true and bot follows the wall
    elif(region["right"] > d and region["center"] < d and region["left"] > d):
        change_state(1)
#case 4 if the obstacle is to the left then turn the bot left till case 3 becomes True then indirectly case 2 becomes True and the bot will follow the wall with its right side nearest to it
    elif(region["right"] > d and region["center"] > d and region["left"] < d):
        change_state(1)
#case 5 if right and center are detecting wall then turn left so that right side is nearest to wall and then follow the wall
    elif(region["right"] < d and region["center"] < d and region["left"] > d):
        change_state(1)
#case 6 this is intermediate step while performing case 4
    elif(region["right"] > d and region["center"] < d and region["left"] < d):
        change_state(1)
#case 7 if the bot in between 2 obstacles then find the wall i.e start turning right so that case 2 becomes true and you follow the right wall
    elif(region["right"] < d and region["center"] > d and region["left"] < d):
        change_state(0)

def find_wall():
    global message
    message.linear.x = 0.4
    message.angular.z = -0.6
    pub.publish(message)

def go_left():
    global message
    message.linear.x = 0
    message.angular.z = 0.3
    pub.publish(message)

def follow_wall():
    global message
    message.linear.x = 0.3
    message.angular.z = 0
    pub.publish(message)


def callback(msg):
    global region
    region ={
        "eright" : min(min(msg.ranges[0:111]) , 2 ),
        "right" :  min(min(msg.ranges[112:287]) , 2 ),
        "center" : min(min(msg.ranges[288:431]) , 2 ),
        "left" :   min(min(msg.ranges[432:609]) , 2 ),
        "eleft" : min(min(msg.ranges[610:719]) , 2 )
    }

    do_task()

def main():
    global pub
    rospy.init_node("wall_follow")
    pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)
    sub = rospy.Subscriber("/m2wr/laser/scan" , LaserScan , callback)

    srv = rospy.Service("/serv_wall_follow" , SetBool , wall_follow_handler)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not activate:
            continue
        else:
            if ( state == 0 ):
                find_wall()
            elif( state == 1 ):
                go_left()
            elif( state == 2):
                follow_wall()
        rate.sleep()


if __name__ == '__main__':
    main()
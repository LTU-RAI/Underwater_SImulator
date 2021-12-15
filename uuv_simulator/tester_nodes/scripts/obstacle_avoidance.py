#!/usr/bin/env python

import rospy
from nav_msgs import msg
from rospy.client import init_node
from uuv_sensor_ros_plugins_msgs import msg
import yaml
import sys
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from rospy.timer import Rate
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range
from uuv_sensor_ros_plugins_msgs.msg import DVLBeam, DVL


range=0
altitude=0

def dvl_altitude(msg):

    global altitude

    altitude=msg.altitude

    return
rospy.init_node('tester_node')
dvl_sub_altitest = rospy.Subscriber('/rexrov/dvl', DVL, dvl_altitude)

if math.fabs(altitude) <= (5+0.556):

                if math.fabs(vector_z) <= 2 and math.fabs(vector_x) <= 2 and math.fabs(vector_y) <= 2:

                    print ("Inspecting at position [" + str(pose.x) + ", " + str(pose.y) + ", " + str(pose.z) + "]")
                    move.linear.x=0
                    move.linear.z=0
                    move.angular.z=0
                    cmd_vel_pub.publish(move) 
                    time.sleep(10) #10 seconds inspection
                    i=i+1
                    j=j+1
                    break


                else: 

                    move.linear.z=0
                    print "too close in z, recalculating route"
                    time.sleep(2)
                    move.linear.z=0.5
                    cmd_vel_pub.publish(move)  
                    time.sleep(3)


                cmd_vel_pub.publish(move)   

            else:

                move.linear.z=linear_vel_z
                

            cmd_vel_pub.publish(move)
rospy.init_node('obstacle_avoidance')
cmd_vel_pub= rospy.Publisher('/rexrov/cmd_vel',Twist,queue_size=1)
#dvl_sub_alti0 = rospy.Subscriber('/rexrov/dvl_sonar0', Range, dvl_altitude0)
dvl_sub_altitest = rospy.Subscriber('/rexrov/dvl', DVL, dvl_altitude)
move= Twist()


i=0

rate=rospy.Rate(10) #10Hz

while not rospy.is_shutdown():


    altitude()



    #print "here"

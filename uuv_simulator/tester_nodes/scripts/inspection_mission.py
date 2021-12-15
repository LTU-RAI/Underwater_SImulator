#!/usr/bin/env python

from yaml.tokens import FlowMappingStartToken
from nav_msgs import msg
import rospy
import yaml
import time
import sys
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range
from uuv_sensor_ros_plugins_msgs.msg import DVLBeam, DVL

def waypoint_loader(filepath):

    with open(filepath, "r") as file_descriptor:
        data= yaml.load(file_descriptor)
    return data

def is_negative(number):

    if number < 0:
        number = bool(True)
    elif number > 0:
        number = bool(False)
    else:
        number= bool(True)
    return number

pose= Point()

def odom_position(msg):
    global pose

    pose=msg.pose.pose.position


    return

roll=pitch=yaw=0

def odom_angle(msg):

    global roll, pitch, yaw

    rot= msg.pose.pose.orientation

    (roll, pitch, yaw)= euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])


def calculate_yaw_angle(vector_x, vector_y):

    
    target_angle_yaw= math.atan2(vector_y, vector_x)



    return target_angle_yaw

angular_vel_yaw=0

def velocity_yaw(error_yaw):

    global angular_vel_yaw

    max_vel=0.5

    error_yaw = np.mod(error_yaw + math.pi, 2*math.pi) - math.pi
    
    if math.fabs(error_yaw) > (math.pi/60):

        if error_yaw >= 0:

            angular_vel_yaw= max_vel

        else:

            angular_vel_yaw=-max_vel
 

    if math.fabs(error_yaw) <= (math.pi/60):

            angular_vel_yaw=0

    return angular_vel_yaw

angular_vel_pitch=0
error_pitch_total=0

def z_pid(vector_z):

    global angular_vel_pitch, error_pitch_total
    max_vel=0.5

    vector_test=is_negative(vector_z)

    if vector_test == True:

        linear_vel_z=-max_vel

    else:

        linear_vel_z=max_vel

    if abs(vector_z) < 0.1:

        linear_vel_z = 0


    return linear_vel_z

range_pos_x=0

def sensor_range(msg):

    global range_pos_x

    range_pos_x=msg.range

    return


coast_clear=False

def clear(range_pose_x):

    global coast_clear

    if range_pos_x >= 4:

        coast_clear=True

    else:

        coast_clear=False

    return coast_clear


def altitude_z(vector_z, vector_x, vector_y, j):




    return j


rospy.init_node('check_odometry')
odom_sub_pose = rospy.Subscriber('/rexrov/pose_gt', Odometry, odom_position)
odom_sub_ang = rospy.Subscriber('/rexrov/pose_gt', Odometry, odom_angle)
cmd_vel_pub= rospy.Publisher('/rexrov/cmd_vel',Twist,queue_size=1)
dvl_sub_altitest = rospy.Subscriber('/rexrov/sensor/sensor_front', Range, sensor_range)
move= Twist()

filepath = "/home/filip/catkin_ws/src/uuv_simulator/tester_nodes/config/waypoint.yaml"
data = waypoint_loader(filepath)

    # find total number of waypoints
total=len(data["waypoints"])

rate=rospy.Rate(10) #10Hz

while not rospy.is_shutdown():

    i=0
    j=0
    move.linear.x = 0
    move.linear.y = 0
    move.linear.z = 0
    move.angular.x=0
    move.angular.y=0
    move.angular.z=0
    cmd_vel_pub.publish(move)

    for i in range(total):

        while not rospy.is_shutdown() and j < total :

            max_vel=0.5
            waypoint = Point()
            waypoint.x=float(data["waypoints"][j][0])
            waypoint.y=float(data["waypoints"][j][1])
            waypoint.z=float(data["waypoints"][j][2])

            vector_x=waypoint.x-pose.x
            vector_y=waypoint.y-pose.y
            vector_z=waypoint.z-pose.z

            target_angle_yaw=calculate_yaw_angle(vector_x, vector_y)
            error_yaw= target_angle_yaw - yaw
            angular_vel_yaw= velocity_yaw(error_yaw)
            move.angular.z=angular_vel_yaw
            cmd_vel_pub.publish(move)

            error_position=math.sqrt(pow(vector_x,2)+pow(vector_y,2))
            
            if error_position > 0.3:

                move.linear.x=max_vel
                cmd_vel_pub.publish(move)

            linear_vel_z= z_pid(vector_z)
            move.linear.z=linear_vel_z
            cmd_vel_pub.publish(move)

            if math.fabs(range_pos_x) <= (1+0.1):

                if math.fabs(vector_z) <= 3 and math.fabs(vector_x) <= 3 and math.fabs(vector_y) <= 3:

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
                    
                    move.linear.x=0
                    move.linear.z=0
                    print "too close in x, recalculating route"
                    time.sleep(2)
                    coast_clear=clear(range_pos_x)

                    while coast_clear == False:

                        move.linear.x=-max_vel
                        cmd_vel_pub.publish(move)

                        coast_clear=clear(range_pos_x)


                cmd_vel_pub.publish(move)
                

            cmd_vel_pub.publish(move)

            if vector_y <= 0.15 and vector_y >= -0.15 and vector_x <= 0.15 and vector_x >= -0.15:

                move.linear.x = 0
                move.linear.y = 0
                move.angular.x=0
                move.angular.y=0
                move.angular.z=0
                cmd_vel_pub.publish(move)

            if  abs(vector_z) < 0.15:
                
                move.linear.z = 0
                cmd_vel_pub.publish(move)

            if vector_y <= 0.15 and vector_y >= -0.15 and vector_x <= 0.15 and vector_x >= -0.15 and vector_z <= 0.15 and vector_z >= -0.15:

                print ("Reached waypoint [" + str(waypoint.x) + ", " + str(waypoint.y) + ", " + str(waypoint.z) + "]")
                i=i+1
                j=j+1

        if j == total:

            print "route complete"
            move.linear.x = 0
            move.linear.y = 0
            move.linear.z = 0
            move.angular.x=0
            move.angular.y=0
            move.angular.z=0
            cmd_vel_pub.publish(move)

            exit()



        #move.linear.x=vel
        #cmd_vel_pub.publish(move)
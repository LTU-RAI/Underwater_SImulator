#!/usr/bin/env python

from nav_msgs import msg
import rospy
import yaml
import sys
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

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

current_pose_x=current_pose_y=current_pose_z=0

def odom_position(msg):
    global current_pose_x, current_pose_y, current_pose_z

    current_pose_x=float(msg.pose.pose.position.x)
    current_pose_y=float(msg.pose.pose.position.y)
    current_pose_z=float(msg.pose.pose.position.z)

    return

#current_angle_x=current_angle_y=current_angle_z=0
roll=pitch=theta=0

def odom_angle(msg):
    #global current_angle_x, current_angle_y, current_angle_z
    global roll, pitch, theta

    rot= msg.pose.pose.orientation

    (roll, pitch, theta)= euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

    #print ("pitch: "+ str(pitch))
    #print theta

    #current_angle_x=float(msg.pose.pose.orientation.x)
    #current_angle_y=float(msg.pose.pose.orientation.y)
    #current_angle_z=float(msg.pose.pose.orientation.z)

def calculate_theta_angle(vector_x, vector_y):

    target_angle_theta= math.atan2(vector_y, vector_x)

    #print "Theta_angle"

    return target_angle_theta

def calculate_pitch_angle(waypoint_x, waypoint_y, waypoint_z, current_pose_x, current_pose_y, current_pose_z):

    num=(waypoint_x * waypoint_x)+(waypoint_y * waypoint_y)+(waypoint_z*current_pose_z)
    a=math.sqrt((waypoint_x*waypoint_x) + (waypoint_y*waypoint_y) + (waypoint_z*waypoint_z))
    b=math.sqrt((waypoint_x*waypoint_x) + (waypoint_y*waypoint_y) + (current_pose_z*current_pose_z))
    den=a*b

    target_angle_pitch= math.acos((num/den))

    return target_angle_pitch


error_xy_total=0
angular_vel_theta=0

def xy_pid(target_angle_theta):

    #global error_xy_total
    global angular_vel_theta

    #i_pid=0
    #last_error=0
    max_vel=0.7

    #kp= 0.5
    #ki= 0.01
    #kd= 0.1


    #while i_pid < 1:

        #error_xy=target_angle_theta - theta

        #if error_xy < 10 and error_xy != 0:

            #error_xy_total += error_xy

        #else:

            #error_xy_total = 0
            #i=1

        #if error_xy == 0:

            #kd=0

        #if error_xy_total > 50/ki:

            #error_xy_total =50/ki

        
        #xy_P = error_xy * kp
        #xy_I = error_xy_total * ki
        #xy_D = (error_xy - last_error) * kd 

        #last_error= error_xy

        #angular_vel_theta= xy_P + xy_I + xy_D



    if abs(target_angle_theta - theta ) > 0.2:

        angular_vel_theta= max_vel

    else:

        angular_vel_theta = 0

    return angular_vel_theta

angular_vel_pitch=0
error_pitch_total=0

def z_pid(target_angle_pitch):

    global angular_vel_pitch, error_pitch_total
    max_vel=0.5
    i_pid=0
    last_error_pitch=0
    kp= 0.2
    ki= 10
    kd= 0.1




    error_pitch=target_angle_pitch + pitch

    print pitch

    #print error_pitch

    #if error_pitch < 10 and error_pitch != 0:

        #error_pitch_total += error_pitch
        #print "test 1"

    #else:

        #error_pitch_total = 0
        #print "test2"

    #if error_pitch == 0:
            
        #print "test 3"
        #kd=0

    #if error_pitch_total > 10/ki or error_pitch_total< -10/ki:
            
        #print "test4"
        #error_pitch_total =10/ki

        
    #pitch_P = error_pitch * kp

    #print ("P = " + str(pitch_P))
    #pitch_I = error_pitch_total * ki

    #print ("I = " + str(pitch_I))
    #pitch_D = (error_pitch - last_error_pitch) * kd 

    #print ("D = " + str(pitch_D))

    #last_error_pitch= error_pitch

    #angular_vel_pitch= (pitch_P + pitch_I + pitch_D)

    angular_vel_pitch=error_pitch

    return angular_vel_pitch


rospy.init_node('check_odometry')
odom_sub_pose = rospy.Subscriber('/rexrov/pose_gt', Odometry, odom_position)
odom_sub_ang = rospy.Subscriber('/rexrov/pose_gt', Odometry, odom_angle)
cmd_vel_pub= rospy.Publisher('/rexrov/cmd_vel',Twist,queue_size=1)
move= Twist()

filepath = "waypoint.yaml"
data = waypoint_loader(filepath)

    # find total number of waypoints
total=len(data["waypoints"])



while not rospy.is_shutdown():

    i=0

    for i in range(total):


        j=0

        while j < total:

            waypoint_x=float(data["waypoints"][j][0])
            waypoint_y=float(data["waypoints"][j][1])
            waypoint_z=float(data["waypoints"][j][2])

            vector_x=waypoint_x-current_pose_x
            vector_y=waypoint_y-current_pose_y

            target_angle_theta=calculate_theta_angle(vector_x, vector_y)

            angular_vel_theta= xy_pid(target_angle_theta)

            move.angular.z=angular_vel_theta
            cmd_vel_pub.publish(move)

            if angular_vel_theta == 0:

            #klart fram hit, do PID for pitch and x-movement, when thats done implement PID for theta

                target_angle_pitch= calculate_pitch_angle (waypoint_x, waypoint_y, waypoint_z, current_pose_x, current_pose_y, current_pose_z)

                angular_vel_pitch= z_pid(target_angle_pitch)

                move.angular.y=-angular_vel_pitch

                #print angular_vel_pitch

                cmd_vel_pub.publish(move)


        
            if angular_vel_theta == 0 and angular_vel_pitch < 0.05:

                move.linear.x=0.5
                cmd_vel_pub.publish(move)
                #print "moving towards waypoint"
                
            if abs(waypoint_y- current_pose_y) < 0.1 and abs(waypoint_z - current_pose_z) < 0.1 and abs(waypoint_x - current_pose_x) < 0.1:
                    
                move.linear.x = 0
                move.linear.y = 0
                move.linear.z = -0.06
                move.angular.x=0
                move.angular.y=0
                move.angular.z=0
                cmd_vel_pub.publish(move)

                print "Reached waypoint"
                i=i+1
                j=j+1


    else:

        print "route complete"


        #move.linear.x=vel
        #cmd_vel_pub.publish(move)
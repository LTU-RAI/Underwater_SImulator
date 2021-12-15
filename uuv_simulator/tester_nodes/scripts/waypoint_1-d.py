#!/usr/bin/env python

from nav_msgs import msg
import rospy
import yaml
import sys
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def waypoint_loader(filepath):

    with open(filepath, "r") as file_descriptor:
        data= yaml.load(file_descriptor)
    return data

pose_x=pose_y=pose_z=0

def odom_position(msg):
    global pose_x, pose_y, pose_z
    pose_x=float(msg.pose.pose.position.x)
    pose_y=float(msg.pose.pose.position.y)
    pose_z=float(msg.pose.pose.position.z)

    return

def is_negative(number):

    if number < 0:

        number = bool(True)

    elif number > 0:

        number = bool(False)

    else:

        number= bool(True)

    return number

def calculate_vel(waypoint_x,waypoint_y,waypoint_z, pose_x, pose_y, pose_z)

    vel_max=0.3
    vel_min=-0.3

    Direction_x=is_negative(waypoint_x - pose_x)
    Direction_y=is_negative(waypoint_y - pose_y)
    Direction_z=is_negative(waypoint_z - pose_z)

    distance_x=abs(waypoint_x - pose_x)
    distance_y=abs(waypoint_y-pose_y)
    distance_z=abs(waypoint_z - pose_z)


    if distance_x > distance_y and distance_x > distance_z:

        if


    elif distance_y > distance_x and distance_y > distance_z:


    else:




        return vel_x, vel_y, vel_z

def calculate_angle()


rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/rexrov/pose_gt', Odometry, odom_position)
cmd_vel_pub= rospy.Publisher('/rexrov/cmd_vel',Twist,queue_size=1)
move= Twist()

filepath = "waypoint.yaml"
data = waypoint_loader(filepath)

    # find total number of waypoints
total=len(data["waypoints"])
    
i=0
k=0

while not rospy.is_shutdown():



    while i < total:

        waypoint_x=float(data["waypoints"][i][0])
        waypoint_y=float(data["waypoints"][i][1])
        waypoint_z=float(data["waypoints"][i][2])
        move.angular.x=0
        move.angular.y=0
        move.angular.z=0
       # move.linear.y=0
       # move.linear.z=0
        cmd_vel_pub.publish(move)
        vel = 1.0
        stop = 0.0
        
        direction_x=waypoint_x - pose_x
        direction_y=waypoint_y-pose_y
        direction_z=waypoint_z-pose_z
        
        direction_x= is_negative(direction_x)
        direction_y= is_negative(direction_y)
        direction_z= is_negative(direction_z)

        #if x-direction is Positive

        if direction_x == False:
        
            if pose_x < waypoint_x - 0.1:

                move.linear.x=vel
                cmd_vel_pub.publish(move)
                
                

            else:

                move.linear.x = stop
                cmd_vel_pub.publish(move)
                k=i
                x_pos=k
                continue
                

        #if x-direction is negative


        else:

            if pose_x > waypoint_x + 0.1:

                move.linear.x=-vel
                cmd_vel_pub.publish(move)

            else:

                move.linear.x=stop
                cmd_vel_pub.publish(move)
                k=i
                x_pos=k
                continue

        #if y-direction is negative

        if direction_y == False:
        
            if pose_y < waypoint_y - 0.1:

                move.linear.y=vel
                cmd_vel_pub.publish(move)
                

            else:

                move.linear.y = stop
                cmd_vel_pub.publish(move)
                k=i
                y_pos=k
                continue
                


        #if y-direction is negative

        else:

            if pose_y > waypoint_y + 0.1:

                move.linear.y=-vel
                cmd_vel_pub.publish(move)

            else:

                move.linear.y=stop
                cmd_vel_pub.publish(move)
                k=i
                y_pos=k
                continue

        #if z-direction is negative

        if direction_z == False:
        
            if pose_z < waypoint_z - 0.1:

                move.linear.z=vel
                cmd_vel_pub.publish(move)
                

            else:

                move.linear.z = stop
                cmd_vel_pub.publish(move)
                k=i
                z_pos=k
                continue
                


        #if y-direction is negative

        else:

            if pose_z > waypoint_z + 0.1:

                move.linear.z=-vel
                cmd_vel_pub.publish(move)

            else:

                move.linear.z=stop
                cmd_vel_pub.publish(move)
                k=i
                z_pos=k
                continue

    if x_pos == k and pos_y == k and pos_z == k:

        print ("framme vid waypoint" + int(i+1))

    else:

        continue


    
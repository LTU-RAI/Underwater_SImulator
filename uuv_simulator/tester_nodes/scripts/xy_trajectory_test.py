#!/usr/bin/env python

from nav_msgs import msg
import rospy
import yaml
import sys
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
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

pose= Point()

def odom_position(msg):
    global pose

    pose=msg.pose.pose.position


    return

#current_angle_x=current_angle_y=current_angle_z=0
roll=pitch=yaw=0

def odom_angle(msg):
    #global current_angle_x, current_angle_y, current_angle_z
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


rospy.init_node('check_odometry')
odom_sub_pose = rospy.Subscriber('/rexrov/pose_gt', Odometry, odom_position)
odom_sub_ang = rospy.Subscriber('/rexrov/pose_gt', Odometry, odom_angle)
cmd_vel_pub= rospy.Publisher('/rexrov/cmd_vel',Twist,queue_size=1)
move= Twist()

filepath = "/home/filip/catkin_ws/src/uuv_simulator/tester_nodes/config/waypoint.yaml"
data = waypoint_loader(filepath)

    # find total number of waypoints
total=len(data["waypoints"])



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

        while j < total:

            max_vel=0.5
            waypoint = Point()
            waypoint.x=float(data["waypoints"][j][0])
            waypoint.y=float(data["waypoints"][j][1])
            waypoint.z=float(data["waypoints"][j][2])

            target = Point()
            target.x=0
            target.y=0
            target.z=-5

            target_vector_x=target.x-pose.x
            target_vector_y=target.y-pose.y
            target_vector_z=target.z-pose.z


            target_angle_yaw=calculate_yaw_angle(target_vector_x, target_vector_y)
            error_target_yaw= target_angle_yaw - yaw
            angular_vel_yaw= velocity_yaw(error_target_yaw)
            move.angular.z=angular_vel_yaw
            cmd_vel_pub.publish(move)

            vector_x=waypoint.x-pose.x
            vector_y=waypoint.y-pose.y
            vector_z=waypoint.z-pose.z

            target_angle=calculate_yaw_angle(vector_x, vector_y)
            error_target_angle= target_angle - yaw
            error_target_angle = np.mod(error_target_angle + math.pi, 2*math.pi) - math.pi


            #print yaw

            error_position=math.sqrt(pow(vector_x,2)+pow(vector_y,2))

            if error_position > 0.3 and math.fabs(error_target_angle) >0.1:

                move.linear.x=max_vel*math.cos(error_target_angle)
                move.linear.y=max_vel*math.sin(error_target_angle)



                cmd_vel_pub.publish(move)



            linear_vel_z= z_pid(vector_z)

            move.linear.z=linear_vel_z

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
#!/usr/bin/env python
import rospy
import numpy as np
import sys

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D,Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan

from time import sleep
Distance = 0
Angle = 0
offset = 0.4
Commande = Twist()
kp_d = 0.3
kd_d = -0.01

kp_a = 1.0
kd_a = -0.2

x = 0
y = 0
theta = 0

V_MAX = 0.1
OMEGA_MAX = 0.5

List_edge = [ [2,-2],[-2,-2],[-2,2],[2,2] ]
indice = 0
Research_objectif = 0
Initial_edge = 0

lidar_ranges = 0 
lidar_angle  = 0 

def callback(data):
    global Distance
    Distance = data.x
    if Distance > 70:
        Distance = 0
    global Angle
    Angle = data.theta

def pid_callback(data):
    global kp_d
    kp_d = data.x

    global kd_d
    kp_d = data.y

    global kp_a
    kp_d = data.z

    global kd_a
    kp_d = data.w

def pose_callback(data):
    global x
    x = data.x

    global y
    y = data.y

    global theta
    theta = data.theta

def LaserScan_callback(data):
    global lidar_ranges,lidar_angle
    lidar_ranges = np.min(data.ranges)
    lidar_angle  = np.argmin(data.ranges)*1.0
    lidar_angle = data.angle_min + (data.angle_max - data.angle_min) * (lidar_angle/len(data.ranges)*1.0)


def end_mission(Commande, pub):
    Commande.linear.x = 0
    Commande.angular.z = 0
    pub.publish(Commande)
    sleep(5)
    print("All weeds have been destroyed. End of the program")
    sys.exit(0)


def regulation():
    rospy.init_node("regulation",anonymous=True)

    pub  = rospy.Publisher("cmd_vel",Twist, queue_size=10)
    goal = rospy.Publisher("OnZone",Bool, queue_size=10)
    rate = rospy.Rate(10)


    rospy.Subscriber("DistAngle", Pose2D,callback)
    rospy.Subscriber("PID",Quaternion,pid_callback)
    rospy.Subscriber("/laser_scan" , LaserScan , LaserScan_callback)

    rospy.Subscriber("Estimated_Position",Pose2D,pose_callback) 

    Research_objectif = 0
    while not rospy.is_shutdown():
        global Commande
        global indice
        global lidar_ranges,lidar_angle
        stop = 0

        # Assert the robot stay in the square [ [4.2,4.2],[4.2,-4.2],[-4.2,-4.2],[-4.2,4.2] ]

        if Distance == 0:
            obj_x = List_edge[indice][0]
            obj_y = List_edge[indice][1]
            dist_edge = ( (x-obj_x)**2 + (y-obj_y)**2 )**0.5
            if dist_edge<0.5:
                if not Research_objectif:
                    Research_objectif = 1
                    Initial_edge = indice
                indice=(indice+1)%4
                print(Initial_edge)
                print(indice)
                if Initial_edge == indice:
                    end_mission(Commande, pub)
            angle = -np.arctan2([obj_y-y],[obj_x-x]) + theta
            
            v  = kp_d * dist_edge
            angle = (angle + np.pi)%(2*np.pi) - np.pi
            tp = kp_a * angle + kd_a * Commande.angular.z



        else:
            Research_objectif = 0
            if Distance <= offset :
                v  = 0
                tp = kp_a * Angle + kd_a * Commande.angular.z
                if tp <= 0.01:
                    tp = 0
                    stop = 1
            else:
                v  = kp_d * (Distance - offset) + kd_d * Commande.linear.x
                tp = kp_a * Angle + kd_a * Commande.angular.z
        
        if lidar_ranges<0.3:
            v = np.sin(lidar_angle)*V_MAX
            tp = (np.pi-lidar_angle+np.pi)%(2*np.pi)-np.pi
        
        Commande.linear.x = min(v,V_MAX)
        Commande.angular.z = min(max(tp,-OMEGA_MAX),OMEGA_MAX)


        goal.publish(stop)
        pub.publish(Commande)
        rate.sleep()

if __name__=="__main__":
    regulation()
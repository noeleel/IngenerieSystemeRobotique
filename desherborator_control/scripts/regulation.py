#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Quaternion

Distance = 0
Angle = 0
offset = 0.2598
Commande = Twist()
kp_d = 0.1
kd_d = -0.01

kp_a = 0.5
kd_a = -0.1

x = 0
y = 0
theta = 0

V_MAX = 0.25
OMEGA_MAX = 0.5

List_edge = [ [3,3],[3,-3],[-3,-3],[-3,3] ]
indice = 0
Research_objectif = 0
Initial_edge = 0

def callback(data):
    global Distance
    Distance = data.x
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




def regulation():
    rospy.init_node("regulation",anonymous=True)

    pub  = rospy.Publisher("cmd_vel",Twist)
    goal = rospy.Publisher("OnZone",Bool)
    rate = rospy.Rate(10)


    rospy.Subscriber("DistAngle", Pose2D,callback)
    rospy.Subscriber("PID",Quaternion,pid_callback)

    rospy.Subscriber("/gps/fixed",Pose2D,pose_callback) 

    
    while not rospy.is_shutdown():
        global Commande
        global indice
        stop = 0

        if Distance == 0:
            obj_x = List_edge[indice][0]
            obj_y = List_edge[indice][1]
            dist_edge = ( (x-obj_x)**2 + (y-obj_y)**2 )**0.5

            if dist_edge<offset:
                if not Research_objectif:
                    Research_objectif = 1
                    Initial_edge = indice
                indice=(indice+1)%4

                if Initial_edge == indice:
                    break
            angle = np.arctan2([obj_y-y],[obj_x-x])

            v  = kp_d * dist_edge
            tp = kp_a * angle


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
        
        Commande.linear.x = min(v,V_MAX)
        Commande.angular.z = min(max(tp,-OMEGA_MAX),OMEGA_MAX)


        goal.publish(stop)
        pub.publish(Commande)
        rate.sleep()

if __name__=="__main__":
    regulation()
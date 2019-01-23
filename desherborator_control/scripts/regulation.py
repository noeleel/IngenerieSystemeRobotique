#!/usr/bin/env python
import rospy
import numpy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D,Pose
from geometry_msgs.msg import Quaternion

Distance = 0
Angle = 0
offset = 0.2598
Commande = Twist()
kp_d = 0.1
kd_d = -0.01

kp_a = 0.5
kd_a = -0.1

V_MAX = 0.25
OMEGA_MAX = 0.5


def callback(data):
    global Distance
    Distance = data.x
    global Angle
    Angle = data.theta

def pid_callback(data):
    global kp_d
    kp_d = data.x

    global kd_d
    kp_d = data.x

    global kp_a
    kp_d = data.x

    global kd_a
    kp_d = data.w

def pose_callback(data):
    global x
    x = data.position.x

    global y
    y = data.position.y

    global theta
    theta = data.orientation.z





def regulation():
    rospy.init_node("regulation",anonymous=True)

    pub  = rospy.Publisher("cmd_vel",Twist)
    goal = rospy.Publisher("OnZone",Bool)
    rate = rospy.Rate(10)


    rospy.Subscriber("DistAngle", Pose2D,callback)
    rospy.Subscriber("PID",Quaternion,pid_callback)
    rospy.Subscriber("pose_robot",Pose,pose_callback) 

    
    while not rospy.is_shutdown():
        global Commande
        stop = 0

        if Distance == 0:
            x  = 0
            theta = 1
        else:
            if Distance <= offset :
                x  = 0
                theta = kp_a * Angle + kd_a * Commande.angular.z
                if theta <= 0.01:
                    theta = 0
                    stop = 1
            else:
                x  = kp_d * (Distance - offset) + kd_d * Commande.linear.x
                theta = kp_a * Angle + kd_a * Commande.angular.z
        
        Commande.linear.x = min(x,V_MAX)
        Commande.angular.z = min(max(theta,-OMEGA_MAX),OMEGA_MAX)


        goal.publish(stop)
        pub.publish(Commande)
        rate.sleep()

if __name__=="__main__":
    regulation()

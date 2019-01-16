#!/usr/bin/env python
import rospy
import numpy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

Distance = 0
Angle = 0
offset = 0.2598
Commande = Twist()
kp_d = 1
kd_d = -0.1

kp_a = 1
kd_a = -0.1


def callback(data):
    global Distance
    Distance = data.x
    global Angle
    Angle = data.theta



def regulation():

    pub = rospy.Publisher("cmd_vel",Twist)
    rospy.init_node("regulation",anonymous=True)
    rate = rospy.Rate(10)


    rospy.Subscriber("DistAngle", Pose2D,callback)
    
    while not rospy.is_shutdown():
        global Commande

        if Distance == 0:
            Commande.linear.x  = 0
            Commande.angular.z = 1
        else:
            Commande.linear.x  = kp_d * (Distance - offset) + kd_d * Commande.linear.x
            Commande.angular.z = kp_a * Angle + kd_a * Commande.angular.z

        pub.publish(Commande)
        rate.sleep()

if __name__=="__main__":
    regulation()
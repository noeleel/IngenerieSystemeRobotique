#!/usr/bin/env python  
import rospy

import time
import math
import numpy as np
import tf2_ros

from sensor_msgs.msg import LaserScan
from  geometry_msgs.msg import Pose2D, Pose, TransformStamped
from tf.transformations import euler_from_quaternion

msg = Pose()
estimated = Pose2D()
euler_msg = [0,0,0]

def eulerAnglesToRotationMatrix(phi, theta, psi):
    R_x = np.array([[              1,              0,                0],
                    [              0,  math.cos(phi),   -math.sin(phi)],
                    [              0,  math.sin(phi),    math.cos(phi)]])

    R_y = np.array([[math.cos(theta),              0, -math.sin(theta)],
                    [              0,              1,                0],
                    [math.sin(theta),              0,  math.cos(theta)]])

    R_z = np.array([[  math.cos(psi), -math.sin(psi),                0],
                    [  math.sin(psi),  math.cos(psi),                0],
                    [              0,              0,                1]])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

def pose_robot_callback(data):
    global msg, euler_msg
    msg.position.x = data.x
    msg.position.y = data.y
    euler_msg = [0,0,data.theta]


time.sleep(5)
if __name__ == '__main__':
    rospy.init_node('tf2_map_to_pose')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('/Estimated_Position', Pose2D, queue_size=10)
    rospy.Subscriber("/Estimated_Position" , Pose2D , pose_robot_callback)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('odom', 'map', rospy.Time())
            temp = trans.transform.rotation
            x,y,z,w = temp.x,temp.y,temp.z,temp.w
            phi, theta, psi = euler_from_quaternion([x,y,z,w])
            R = eulerAnglesToRotationMatrix(phi, theta, psi)
            temp = trans.transform.translation
            x_trans, y_trans, z_trans = temp.x,temp.y,temp.z

            X_odom  = np.array([[msg.position.x],[msg.position.y],[msg.position.z]])
            X_trans = np.array([[x_trans],[y_trans],[z_trans]])
            X_map   = X_trans + np.dot(R,X_odom)

            estimated.x     = X_map[0,0]
            estimated.y     = X_map[1,0]
            estimated.theta = np.mod(psi + euler_msg[2], 2*np.pi) 

            pub.publish(estimated)

            print(estimated)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue


        rate.sleep()
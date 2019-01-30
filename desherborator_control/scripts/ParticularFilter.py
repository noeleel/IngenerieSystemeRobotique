#!/usr/bin/python3

import os
import sys

import numpy as np
import math
import matplotlib.pyplot as plt
import time
import signal

import rospy
import laser_geometry.laser_geometry as lg
from std_msgs.msg import String, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D

not_started = True
no_command  = True

NP = 100
NM = 20
RANGE_MIN, RANGE_MAX = -3/4*np.pi, 3/4*np.pi

angles_extremum = []
lidar_ranges    = []
time_increment  = 0

# walls [a, b, c] <=> a*y, b*x, c*cst
walls = [[np.array([[-4.7], [-4.7]]), np.array([[-4.7],[ 4.7]])],
         [np.array([[-4.7], [ 4.7]]), np.array([[ 4.7],[ 4.7]])],
         [np.array([[ 4.7], [ 4.7]]), np.array([[ 4.7],[-4.7]])],
         [np.array([[ 4.7], [-4.7]]), np.array([[-4.7],[-4.7]])]]

vect_walls = [np.array([[ 0.],[ 1.]]), 
              np.array([[ 1.],[ 0.]]), 
              np.array([[ 0.],[-1.]]), 
              np.array([[-1.],[ 0.]])]

vx, vy, vz = 0.0,0.0,0.0     
wx, wy, wz = 0.0,0.0,0.0
DT = 0.001

###################################
#    Callback                     #
###################################

def LaserScan_callback(data):
    global not_started, lidar_ranges, DT, angles_extremum
    angles_extremum = [data.angle_min, data.angle_max]
    lidar_ranges    = data.ranges
    DT              = data.time_increment 
    not_started     = False
    

def Command_callback(data):
    global vx, vy, vz, wx, wy, wz, no_command
    vx = data.linear.x
    vy = data.linear.y
    vz = data.linear.z
    #print("v : ", vx, vy, vz)
    wx = data.angular.x
    wy = data.angular.y
    wz = data.angular.z
    #print("w : ", wx, wy, wz)
    no_command = False

###################################
#    Utilities                    #
###################################

def gauss_likelihood(x, sigma):
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
               math.exp(-x ** 2 / (2 * sigma ** 2))
    return p

def calc_covariance(xEst, px, pw):
    cov = np.matrix(np.zeros((3, 3)))
    for i in range(px.shape[1]):
        dx = (px[:, i] - xEst)[0:3]
        cov += pw[0, i] * dx * dx.T
    return cov

def motion_model(x, u):
    # X = [X,Y,Th,V]
    F = np.matrix([[1.0, 0  , 0  ,  0],
                   [0  , 1.0, 0  ,  0],
                   [0  , 0  , 1.0,  0],
                   [0  , 0  , 0  ,0.0]])
    # U = [VX, WZ]
    B = np.matrix([[DT * math.cos(x[2, 0]),   0],
                   [DT * math.sin(x[2, 0]),   0],
                   [0.0                   ,  DT],
                   [1.0                   , 0.0]])
    #print("#1:\n", u)
    x = F * x + B * u
    #print("#2:\n", x)
    return x

def simulated_measure(px):
    
    measures = np.zeros([NP,NM]) + 50 
    
    for ind_part in range(NP):
        #print("#################################")
        for ind_ray in range(NM):
            
            alpha = RANGE_MIN + ind_ray*(RANGE_MAX-RANGE_MIN)/(NM-1)
            #print("###")
            for ind_w, wall in enumerate(walls):
                vect_wall = vect_walls[ind_w]
                vect_ray = np.array([[np.cos(alpha+px[2,ind_part])], [np.sin(alpha+px[2,ind_part])]])

                p1, p2 = wall[0], wall[1]
                if not (np.abs(vect_ray[0,0]) == np.abs(vect_wall[0,0])):
                    if (vect_wall[1,0]==0): 
                        t = ( wall[0][1,0] - px[0,ind_part] )/ vect_ray[1,0]
                    else:
                        t = (-vect_wall[1,0]/np.linalg.det(np.concatenate((vect_ray, vect_wall), axis=1))) \
                            * (px[0,ind_part] - wall[0][0,0] - (vect_wall[0,0] / vect_wall[1,0])*(px[1,ind_part] - wall[0][1,0]))                        

                    inter_x = vect_ray[0,0]*t + px[0,ind_part]
                    inter_y = vect_ray[1,0]*t + px[1,ind_part]
                    inter = np.array([[inter_x],
                                      [inter_y]])
                    #print("Inter:",inter)
                    #print("Px   :",px[0:2,ind_part])
                    meas = np.linalg.norm(inter-px[0:2,ind_part])
                    if t > 0 and meas<measures[ind_part, ind_ray]:
                        measures[ind_part, ind_ray] = meas
                #print(">>> measure", meas)

    return measures

def check_ranges_to_points(measures, px):
    points = np.zeros((NP, NM, 2))
    for part in range(NP):
        for measure in range(NM):
            alpha = RANGE_MIN + measure*(RANGE_MAX-RANGE_MIN)/(NM-1)
            #print(measures[part, measure])
            x = measures[part, measure]*np.cos(alpha) + px[0,part]
            y = measures[part, measure]*np.sin(alpha) + px[1,part]
            points[part, measure, 0] = x
            points[part, measure, 1] = y
    return points

def check_display(points, px):
    plt.figure()
    plt.axis("equal")
    plt.plot(px[0,:], px[1,:], "+k")

    #print("[{:.2f},{:.2f}]".format(px[0,0], points[0,0,0]))
    #print("[{:.2f},{:.2f}]".format(px[1,0], points[0,0,1]))

    for part in range(NP):
        for measure in range(NM):
            plt.plot([px[0,part],points[part,measure,0]], [px[1,part],points[part,measure,1]], 'r', linewidth=0.4)
    plt.show()

###################################
#    Steps of Filter              #
###################################

def resampling(px, pw):
    Neff = 1.0 / (pw * pw.T)[0, 0]  # Effective particle number
    if Neff < NP//2:
        wcum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / NP) - 1 / NP
        resampleid = base + np.random.rand(base.shape[1]) / NP

        inds = []
        ind = 0
        for ip in range(NP):
            while resampleid[0, ip] > wcum[0, ind]:
                ind += 1
            inds.append(ind)

        px = px[:, inds]
        pw = np.matrix(np.zeros((1, NP))) + 1.0 / NP  # init weight

    return px, pw

###################################
#    Overall of the filter        #
###################################

def pf_localization(px, pw, xEst, PEst, z_sim, z_true, u):
    """
    Localization with Particle filter
    """
    for ip in range(NP):
        x = px[:, ip] # x of particle
        w = pw[0, ip] # w of particle

        #  Predict with ramdom input sampling
        #print("#1\n", x)
        x = motion_model(x, u) # application of the model with noise
        #print("#2\n", x)
        #  Calc Inportance Weight with all the measures available
        for im in range(NM):
            dz = z_true[im] - z_sim[ip, im]             # difference of norm between the particle and the measurement
            w = w * gauss_likelihood(dz, 1)

        #changing the values
        px[:, ip] = x
        pw[0, ip] = w

    pw = pw / pw.sum()  # normalize
    xEst = px * pw.T
    PEst = calc_covariance(xEst, px, pw)

    px, pw = resampling(px, pw)
    return xEst, PEst, px, pw

if __name__=="__main__":
    
    #ROS Initialisation

    rospy.init_node('PF', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rospy.Subscriber("/laser_scan" , LaserScan , LaserScan_callback)
    rospy.Subscriber("/cmd_vel"    ,     Twist ,   Command_callback)

    pub = rospy.Publisher('/Estimated_Position', Pose2D, queue_size=10)

    # State Vector [x y yaw v]'
    xEst  = np.matrix(np.zeros((4, 1)))
    PEst  = np.eye(4)

    px = np.matrix(np.zeros((4, NP)))             # Particle store
    pw = np.matrix(np.zeros((1, NP))) + 1.0 / NP  # Particle weight 
    
    px[0:2,:] = np.random.uniform(-1.0, 1.0, (2,NP))
    px[2,:] = np.random.uniform(-1.54,1.54,(1,NP))

    
    #########################
    # Test                  #
    #########################
    #px[0,:] += 4.
    #px[1,:] += 4.
    #px[2,:] += np.pi/2
    #print(px)
    #measures = simulated_measure(px)
    #points = check_ranges_to_points(measures, px)
    #check_display(points, px)
    #sys.exit()

    while not_started and not rospy.is_shutdown():
        continue


    while not rospy.is_shutdown():
        t0 = time.time()
        u = np.array([[vx],[wz]])
        z_true   = lidar_ranges
        measures = simulated_measure(px)
        xEst, PEst, px, pw = pf_localization(px, pw, xEst, PEst, measures, z_true, u)

        estimated = Pose2D()
        estimated.x = xEst[0,0]
        estimated.y = xEst[1,0]
        estimated.theta = xEst[2,0]

        print(estimated)
        pub.publish(estimated)
        print("Temps Loop : {:.2f}s".format(time.time()-t0))

        rate.sleep()
        



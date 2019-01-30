#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from std_msgs.msg import Bool
from math import sin, cos, atan2, sqrt, fabs
from time import sleep

# Define publishers for joint1 and joint2 position controller commands.joint_chassis_to_dummy
jointSlidingArmPublisher = rospy.Publisher('/desherborator/slidingArm_position_controller/command', Float64, queue_size=10)
jointRotatingArmPublisher = rospy.Publisher('/desherborator/rotatingArm_position_controller/command', Float64, queue_size=10)

# Min and max command for sliding the robot
max_command = 0.1
min_command = 0

#Useful variables
plant_detected = False

def callback(msg):
    global plant_detected
    plant_detected = msg.data

def goToInitialPose():
    jointSlidingArmPublisher.publish(0)
    jointRotatingArmPublisher.publish(0)
    sleep(2)

# Used for sliding the arm up
def slide_up(command_value):
    jointSlidingArmPublisher.publish(command_value)
    sleep(1)
    print "Going up"

# Used for sliding the arm down
def slide_down(command_value):
    jointSlidingArmPublisher.publish(-command_value)
    print "Going down"

# Used for rotating the arm left
def slide_left(command_value):
    jointRotatingArmPublisher.publish(command_value)
    print "Going left"

# Used for rotating the arm right
def slide_right(command_value):
    jointRotatingArmPublisher.publish(-command_value)
    print "Going right"

# Deploy the arm at 60 degrees
def deploy_arm():
    slide_up(max_command)
    sleep(1)
    slide_right(80*(3.14/180))
    sleep(2)
    slide_down(max_command)
    sleep(1)

# Used for waiting until the herb is desintegrated
def wait_desintregration():
    wait_time = 15
    sleep(wait_time)
    print "Herb desintegrated"

# Main file
def main():
    rospy.init_node("move_arm")
    rospy.Subscriber("OnZone", Bool ,callback)
    destroyHerb = rospy.Publisher("HerbDestroyed",Bool, queue_size=10)
    while not rospy.is_shutdown():
        destroyHerb.publish(0)
        goToInitialPose()
        if plant_detected == True:
            deploy_arm()
            wait_desintregration()
            destroyHerb.publish(1)



# Below code that will continuously run (to stop it press CTRL+C)
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin, cos, atan2, sqrt, fabs

# Define publishers for joint1 and joint2 position controller commands.joint_chassis_to_dummy
jointSlidingArmPublisher = rospy.Publisher('/joint_states/joint_chassis_to_dummy/command', Float64, queue_size=10)
jointRotatingArmPublisher = rospy.Publisher('/joint_states/joint_dummy_to_rotating_bras/command', Float64, queue_size=10)


# RRArm initial joint positions publisher for joint controllers.
def rrarm_positions_publisher():
    global jointSlidingArmPublisher, jointRotatingArmPublisher
    positioning = True
    jointSlidingArmPublisherInitPose = 1.0
    jointRotatingArmPublisherInitPose = 2.4
    move_step_joint1 = 0.01
    move_step_joint2 = 0.035
    step_upward1 = 0.02
    step_upward2 = 0.04

    jointSlidingArmPublisherCurrentPose = jointSlidingArmPublisherInitPose
    jointRotatingArmPublisherInitPoseCurrentPose = jointRotatingArmPublisherInitPose

    # Initialization of node for controlling joint1 and joint2 positions.
    rospy.init_node('joint_positions_node', anonymous=True)

    rate = rospy.Rate(80)  # Rate 80 Hz

    while not rospy.is_shutdown() and positioning:
        rrarm_initial_joints_position(jointSlidingArmPublisherInitPose, jointRotatingArmPublisherInitPose)
        rospy.sleep(1)
        jointSlidingArmPublisherCurrentPose, jointRotatingArmPublisherInitPoseCurrentPose = rrarm_roll_back(jointSlidingArmPublisherCurrentPose, jointRotatingArmPublisherInitPoseCurrentPose,
                                                                         step_upward1, step_upward2)
        rospy.sleep(1)
        jointSlidingArmPublisherCurrentPose = rrarm_lift_up(jointSlidingArmPublisherCurrentPose, step_upward1)
        positioning = False
    print "End of positioning!"


def rrarm_initial_joints_position(jointSlidingArmPublisherInitPose, jointRotatingArmPublisherInitPose):
    publish_joints_position(jointSlidingArmPublisherInitPose, jointRotatingArmPublisherInitPose)
    print "%.2f joint1 position:" % (jointSlidingArmPublisherInitPose)
    print "%.2f joint2 position:" % (jointRotatingArmPublisherInitPose)


def rrarm_roll_back(joint1_position, joint2_position, step_upward1, step_upward2):
    for x in range(1, 10):
        joint2_position = joint2_position + step_upward2
        joint1_position = joint1_position - step_upward1
        publish_joints_position(joint1_position, joint2_position)
        rospy.sleep(0.2)
        print "%.2f joint1 position:" % (joint1_position)
    return joint1_position, joint2_position


def rrarm_lift_up(joint1_position, step_upward1):
    for x in range(1, 10):
        joint1_position = joint1_position - step_upward1 * x
        publish_joint_position(joint1_position, 1)
        rospy.sleep(0.2)
        print "%.2f joint1 position:" % (joint1_position)
    return joint1_position


def publish_joints_position(joint1_position, joint2_position):
    jointSlidingArmPublisher.publish(joint1_position)
    jointRotatingArmPublisher.publish(joint2_position)


def publish_joint_position(joint_position, joint_number):
    if joint_number == 1:
        jointSlidingArmPublisher.publish(joint_position)
    elif joint_number == 2:
        jointRotatingArmPublisher.publish(joint_position)


# Below code that will continuously run (to stop it press CTRL+C)
if __name__ == '__main__':
    try:
        rrarm_positions_publisher()
    except rospy.ROSInterruptException:
        pass
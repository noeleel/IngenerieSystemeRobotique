import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import sys
import os

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from sensor_msg.msg import Image
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest


# Calcul the distance to the weed
def dist(h_reel, F,frame):
    # Coverte to hsv
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    HSV = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Create the mask
    lower_green = np.array([50,100,50])
    upper_green = np.array([150, 255, 255])
    mask = cv2.inRange(HSV, lower_green, upper_green)

    # Find contours
    isolated = cv2.bitwise_and(img, img, mask=mask)
    contours, hierachiy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find minimum rectangle
    if len(contours) > 0:
        cnt = max(contours,key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(cnt)
        dist = h_reel * F / h
    else:
        dist = 0
        cnt = [0, 0]

    # Affichage
    #cv2.rectangle(img, (x,y), (x+w, y+h), (0, 255, 255), 2)
    #cv2.imshow("img",img)
    #time.sleep(10)
    return dist, cnt
#Calcul the angle
def WeedAngle(frame, cnt):
    height, width = frame.shape[:2]
    #on parcourt le contour avec y_objet le plus grand   (descend selon y et droite selon x)
    matrice = cnt[:,0,:]
    matrice_y = matrice[:,1]

    indice = np.argmax(matrice_y) #indice du point dont le x_objet est le plus grand
    x_objet = cnt[indice,0,0]
    y_objet = cnt[indice,0,1]

    x_origine = width/2
    y_origine = height

    v1 = np.array([[width/2-x_origine],[0-y_origine]])
    v2 = np.array([[x_objet-x_origine],[y_objet-y_origine]])

    x1 = v1[0,0]
    y1 = v1[1,0]
    x2 = v2[0,0]
    y2 = v2[1,0]

    dot = x1*x2 + y1*y2      # dot product
    det = x1*y2 - y1*x2      # determinant
    angle = arctan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
    return angle
# change status of the weed alive => dead
def modif_couleur(request, xrbt, yrbt):
        PATH = sys.argv[1]
        urdf = ""
        with open(PATH+"box.urdf", "r") as stream:
            urdf = stream.read()

        urdf.replace("{xrbt}", str(xrbt))
        urdf.replace("{yrbt}", str(yrbt))

        request.model_name = "box"
        request.model_xml = urdf
        request.initial_pose.position.x = xrbt
        request.initial_pose.position.y = yrbt
        request.initial_pose.position.z = 0

        response = spawnModelService(request)
        if response.success:
            rospy.loginfo("Spawn_Success")
        else:
            rospy.logwarn(response.status_message)


def cb_cam(msg):
    frame = msg.data

def cb_bool(msg):
    bool_weed_red = msg.data

##########################################################################
#                                                                        #
#                                   MAIN                                 #
#                                                                        #
##########################################################################

### Constantes
h_reel = 0.1
F = 1.3962634   #F = distance_objet(m) * hauteur pixel weed / hauteur reel objet(m)

if __name__ == '__main__':

    rospy.init_node('dist_angle')
    # Publishers
    pub = rospy.Publisher('DistAngle', Pose2D)
    DistAngle = Pose2D()

    # Subscribers
    rospy.Subscriber("/main_camera/image_raw", Image, cb_cam)
    rospy.Subscriber("/bool_action",Bool, cb_bool) #TODO

    # Services
    spawnModelService = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)  # Spawn the boxes
    request = SpawnModelRequest()

    rate = rospy.Rate(25)

    while not rospy.is_shutdown():
        #Calcul varibale
        dist, cnt = dist(h_reel, F, frame)
        theta = WeedAngle(frame, cnt)
        #Publish
        DistAngle.x = dist
        DistAngle.theta = theta
        pub.publish(DistAngle)

        # Changement detat des weeds
        if bool_weed_red:
            modif_couleur(request, xrbt, yrbt)

        rate.sleep()

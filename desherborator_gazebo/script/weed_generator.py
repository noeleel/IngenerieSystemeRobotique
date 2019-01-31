#!/usr/bin/env python

from random import uniform
import sys
import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from math import sqrt
from geometry_msgs.msg import Pose2D, Pose, Twist
from std_msgs.msg import Bool



def generer_herbe (fichier, nombre_plantes=1):

    global dictionary

	rospy.init_node("weed_spawner")
	rospy.wait_for_service("/gazebo/spawn_urdf_model")
	spawnerService = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

	request = SpawnModelRequest()

	with open(fichier, "r") as stream:
		template = stream.read()
	for i in range(nombre_plantes):
		request.model_name = "Weed_{}".format(i)
		urdf = template.format(uniform(0.01, 0.075))
		request.model_xml = urdf
		x_herbe = uniform(-10,10)
		y_herbe = uniform(-10,10)
		z_herbe = 0.1/2

        dictionary["Weed_{}".format(i)] = [x_herbe, y_herbe]

		request.initial_pose.position.x = x_herbe
		request.initial_pose.position.y = y_herbe
		request.initial_pose.position.z = z_herbe
		response = spawnerService(request)
		if not response.success:
			rospy.logwarn("Unable to spawn weed {}: {}".format(i, response.status_message))


def compare_coord(x,y):
	global dictionary
	normes = []
	elements = []

	for element in dictionary:
		coords = dictionary.get(element)
		norme = sqrt((x-coords[0])**2 + (y-coords[1])**2)
		normes.append(norme)
		elements.append(element)

	indice = normes.index(min(normes))
	name_destroyed = elements[indice]

	del dictionary[name_destroyed] #pour supprimer l'entree

	return name_destroyed

def cb_bool(msg):
    global bool_weed_red
    bool_weed_red = msg.data

def cb_pose(msg):
    global x,y
    x = msg.x
	y = msg.y

if __name__ == '__main__':
    #PATH = "/home/haddock/5.8/src/desherborator_gazebo/model/"
	rospy.init_node('gener_destr')

	rospy.Subscriber("WeedDestroyed",Bool, cb_bool)
	rospy.Subscriber("Estimated_position",PoseD, cb_pose)

    dictionary = {}

	#Generation
    PATH = sys.argv[1]
    fichier_modele = PATH+"model.urdf"
    nombre_plantes = 30
    generer_herbe(fichier_modele, nombre_plantes)

	#Suppression
	global bool_weed_red
	if bool_weed_red:
		global x,y
		name = compare_coord(x,y)

		DeleteService = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
		DeleteService(str(name))
		request = SpawnModelRequest()
		print 'Weed Deleted'
		rospy.sleep(5)

	rate.sleep()

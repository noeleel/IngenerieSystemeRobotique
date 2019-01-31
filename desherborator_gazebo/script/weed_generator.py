#!/usr/bin/env python

from random import uniform
import sys
import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse

def generer_herbe (fichier, nombre_plantes=1):
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
		x_herbe = uniform(-25,25)
		y_herbe = uniform(-25,25)
		z_herbe = 0.1/2
		request.initial_pose.position.x = x_herbe
		request.initial_pose.position.y = y_herbe
		request.initial_pose.position.z = z_herbe
		response = spawnerService(request)
		if not response.success:
			rospy.logwarn("Unable to spawn weed {}: {}".format(i, response.status_message))


if __name__ == '__main__':
    #PATH = "/home/haddock/5.8/src/desherborator_gazebo/model/"
    PATH = sys.argv[1]
    fichier_modele = PATH+"model.urdf"
    nombre_plantes = 30
    generer_herbe(fichier_modele, nombre_plantes)

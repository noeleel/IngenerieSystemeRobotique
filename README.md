Un simple repo git pour l'ingénierie système en robotique avec la méthode Agile

Repo Taiga du projet :
https://tree.taiga.io/project/noeleel-noeleelingeneriesystemerobotique/timeline

----------

Avant de lancer le programme, merci d'installer les libraires ros suivantes:
    sous Kinetic:
          sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-hector-gazebo ros-kinetic-gazebo-plugins ros-kinetic-gazebo-msgs

    sous Melodic:
          sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-hector-gazebo ros-melodic-gazebo-plugins ros-melodic-gazebo-msgs
----------
Pour lancer le programme dans sa globalité:

* *roslaunch desherborator_control desherborator.launch*

Pour afficher la carte dans gazebo seul:

* *roslaunch desherborator_gazebo garden.launch*

Pour afficher le robot seul dans gazebo dans *empty_world*:

* *roslaunch desherborator_description desherborator_gazebo_robot.launch*

Pour afficher le robot seul dans *rviz*:

* *roslaunch desherborator_description desherborator_rviz_robot.launch*

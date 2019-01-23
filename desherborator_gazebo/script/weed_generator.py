#!/usr/bin/env python

from random import uniform
import sys
import rospy

def generer_herbe (fichier, nombre_plantes=1):
    if nombre_plantes <= 0 or type(nombre_plantes) is not int:
        #print("nombre_plantes doit etre un nombre entier egal ou plus grand que 1")
        return None
        
        
    else:        
        f = open(fichier, 'w+')
        f.write("<?xml version='1.0'?>\n")
        f.write("<sdf version='1.6'>\n")
        f.write("  <model name='unit_cylinder'>\n")
        
        
        for i in range(nombre_plantes):
            x_herbe = str(uniform(-4.5,4.5))
            y_herbe = str(uniform(-4.5,4.5))
            

            f.write("    <link name='link_4_clone_"+str(i)+"'>\n")
            f.write("      <pose frame=''>"+str(x_herbe)+" "+str(y_herbe)+" 0.01 0 0 -0</pose>\n")
            f.write("      <inertial>\n")
            f.write("        <mass>1</mass>\n")
            f.write("        <inertia>\n")
            f.write("          <ixx>0</ixx>\n")
            f.write("          <ixy>0</ixy>\n")
            f.write("          <ixz>0</ixz>\n")
            f.write("          <iyy>0</iyy>\n")
            f.write("          <iyz>0</iyz>\n")
            f.write("          <izz>0</izz>\n")
            f.write("        </inertia>\n")
            f.write("        <pose frame=''>0 0 0.01 0 -0 0</pose>\n")
            f.write("      </inertial>\n")
            f.write("      <gravity>1</gravity>\n")
            f.write("      <self_collide>0</self_collide>\n")
            f.write("      <kinematic>0</kinematic>\n")
            f.write("      <visual name='visual'>\n")
            f.write("        <pose frame=''>0 0 0.01 0 -0 0</pose>\n")
            f.write("        <geometry>\n")
            f.write("          <cylinder>\n")
            f.write("            <radius>"+str(uniform(0.01,0.075))+"</radius>\n")
            f.write("            <length>0.1</length>\n")
            f.write("          </cylinder>\n")
            f.write("        </geometry>\n")
            f.write("        <material>\n")
            f.write("          <lighting>1</lighting>\n")
            f.write("          <script>\n")
            f.write("            <uri>file://media/materials/scripts/gazebo.material</uri>\n")
            f.write("            <name>Gazebo/Grey</name>\n")
            f.write("          </script>\n")
            f.write("          <shader type='pixel'>\n")
            f.write("            <normal_map>__default__</normal_map>\n")
            f.write("          </shader>\n")
            f.write("          <ambient>0 1 0.3 1</ambient>\n")
            f.write("          <diffuse>0 1 0 1</diffuse>\n")
            f.write("          <specular>0 1 0 1</specular>\n")
            f.write("          <emissive>0 0 0 1</emissive>\n")
            f.write("        </material>\n")
            f.write("        <transparency>0</transparency>\n")
            f.write("        <cast_shadows>1</cast_shadows>\n")
            f.write("      </visual>\n")
            f.write("    </link>\n")

            
            
            
        f.write("    <static>1</static>\n")
        f.write("    <allow_auto_disable>1</allow_auto_disable>\n")
        f.write("  </model>\n")
        f.write("</sdf>")
        f.close()

if __name__ == '__main__':
    #PATH = "/home/haddock/5.8/src/desherborator_gazebo/model/"
    PATH = sys.argv[1]
    fichier_modele = PATH+"model.sdf"
    nombre_plantes = 20
    generer_herbe(fichier_modele, nombre_plantes)

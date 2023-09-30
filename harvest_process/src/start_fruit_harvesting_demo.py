#! /usr/bin/env python

""" ESTE ARCHIVO COMIENZA EL PROCESO DE RECOLECCION COMPLETO [DEMO] """

import os
import rospy
import sys
import yaml
import rospkg

from fruit_harvesting_demo import FruitHarvestingDemo


rospy.init_node('harvest_demo_node', log_level=rospy.DEBUG)

# Ruta de los ficheros que contienen los parametros iniciales:
rospack = rospkg.RosPack()
folder_path = os.path.join(rospack.get_path('harvest_process'), "config")
config_file_path = os.path.join(folder_path, "config.yaml")         # Archivo de configuracion
coord_file_path = os.path.join(folder_path, "object_coord.yaml")    # Archivo con las coordenadas del objeto


# Lectura de los datos de los ficheros para poder usarlos en el constructor del FruitHarvestingDemo
coord = [0.0,0.0,0.0]
robot_dist = None
arm_dist = None
forward_dist = None
backward_dist = None

# Lectura de los datos del archivo de configuracion
with open(config_file_path, "r") as file:
    rospy.loginfo("  ---- PARAMETROS DE CONFIGURACION ----")
    
    data = yaml.safe_load(file)
    
    for item, value in data.items():
        
        if item == "robot_approach_distance":
            robot_dist = value
            rospy.loginfo("Distancia de aproximacion robot-objeto: {}".format(value))

        if item == "arm_approach_distance":
            arm_dist = value
            rospy.loginfo("Distancia de aproximacion brazo-objeto: {}".format(value))

        if item == "forward_distance":
            forward_dist = value
            rospy.loginfo("Distancia de avance para agarrar el objeto: {}".format(value))
        
        if item == "backward_distance":
            backward_dist = value
            rospy.loginfo("Distancia de retroceso para retirar el objeto: {}".format(value))
    

# Lectura de los datos del archivo de coordenadas
with open(coord_file_path, "r") as file:
    rospy.loginfo("  ---- COORDENADAS DEL OBJETO ----")

    data = yaml.safe_load(file)
    
    for item, value in data.items():
        
        if item == "x":
            coord[0] = value
            rospy.loginfo("x: {}".format(value))

        if item == "y":
            coord[1] = value
            rospy.loginfo("y: {}".format(value))

        if item == "z":
            coord[2] = value
            rospy.loginfo("z: {}".format(value))
    


demo = FruitHarvestingDemo(coord, robot_approach_dist = robot_dist, arm_approach_dist = arm_dist, forward_dist = forward_dist, backward_dist = backward_dist, frame = "robot_map", test_arm = False)



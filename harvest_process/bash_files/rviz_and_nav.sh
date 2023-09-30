#!/bin/bash

# Se ejecuta el lanzador del RVIZ desde el portatil para poder visualizarlo
gnome-terminal --geometry 67x10+0-0 -e "bash -c 'roslaunch harvest_process rviz_complete.launch'"

sleep 5

# Se ejecuta el nodo que lee el punto seleccionado en RVIZ para escribirlo en el archivo object_coord.yaml
gnome-terminal --geometry 67x10+0-0 -e "bash -c 'rosrun harvest_process set_object_point.py'"

sleep 5

# Se lanzan todos los procesos para la navegacion: mapa, localizacion y navegacion
gnome-terminal --geometry 67x10+0-0 -e "bash -c 'sshpass -p R0b0tn1K ssh rbkairos@192.168.1.156 roslaunch harvest_process nav_complete.launch'"

sleep 15

exit 0
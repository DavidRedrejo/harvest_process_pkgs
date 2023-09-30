#! /usr/bin/env python

import rospy
import os
import rospkg

from geometry_msgs.msg import PointStamped


topic_name = '/TEST/clicked_point'
rospack = rospkg.RosPack()
folder_path = os.path.join(rospack.get_path('harvest_process'), "config")
object_coord_file_path = os.path.join(folder_path, "object_coord.yaml")


def callback(data):
    rospy.loginfo("Punto seleccionado:")
    print(data)

    point = data.point
    x = point.x
    y = point.y
    z = point.z

    write_point(x,y,z)


def write_point(x,y,z):
    # Escritura del archivo en formato 
    # x: valor
    # y: valor
    # z: valor

    f = open(object_coord_file_path, "w")
    f.write("x: %.3f\ny: %.3f\nz: %.3f" % (x,y,z))


if __name__=='__main__':

    rospy.init_node('object_point_node', anonymous=True)
    rospy.Subscriber(topic_name, PointStamped, callback)    

    rospy.spin()    # mantiene el nodo en ejecucion
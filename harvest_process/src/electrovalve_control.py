#! /usr/bin/env python

""" ESTE ARCHIVO CONTIENE LA CLASE Electrovalve CON TODAS LAS FUNCIONES PARA CONTROLAR LA ELECTROVALVULA """
""" Se importa en fruit_harvesting_demo.py para crear una instancia de la clase """
""" Tambien se puede ejecutar este .py directamente para controlar la electrovalvula con las flechas del mando (funcion pad_listener) """

import rospy

from sensor_msgs.msg import Joy
from ur_msgs.srv import SetIO, SetIORequest


class Electrovalve():
    def __init__(self, pin_num = 7):

        # Variables para controlar la pulsacion del mando y la activacion de la electrovalvula
        self.up_down_button = 0.0   # flecha arriba: 1.0 // flecha abajo: -1.0
        self.is_active = False

        service_name = '/robot/arm/ur_driver/set_io'
        rospy.wait_for_service(service_name)
        self.change_IO_client = rospy.ServiceProxy(service_name, SetIO)

        self.change_IO_request = SetIORequest()
        self.change_IO_request.fun = 1              # Indica la funcion: 1 -> Cambio de estado del pin
        self.change_IO_request.pin = pin_num        # Numero del pin en el que se conecta la electrovalvula
        
        # self.pad_listener()

        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        rospy.loginfo("   Electrovalve shutdown!!")
        self.deactivate()


    def activate(self):
        # Cambia la salida a 1 -> activa la electrovalvula
        rospy.logdebug("Activando") 

        self.change_IO_request.state = 1.0

        result = self.change_IO_client(self.change_IO_request)      # se hace la peticion al cliente para cambiar el estado del pin
        rospy.logdebug(result)

        if result.success:
            rospy.loginfo("Se ha ACTIVADO la electrovalvula correctamente.")
            self.is_active = True
        else:
            self.is_active = False
            rospy.logerr("NO SE HA PODIDO ACTIVAR la electrovalvula.")
        
        return result.success


    def deactivate(self):
        # Cambia la salida a 0 -> desactiva la electrovalvula
        rospy.logdebug("Desactivando") 

        self.change_IO_request.state = 0.0

        result = self.change_IO_client(self.change_IO_request)      # se hace la peticion al cliente para cambiar el estado del pin
        rospy.logdebug(result)
        
        if result.success:
            rospy.loginfo("Se ha DESACTIVADO la electrovalvula correctamente.")
            self.is_active = False
        else:
            self.is_active = True
            rospy.logerr("NO SE HA PODIDO DESACTIVAR la electrovalvula.")

        return result.success


    def get_state(self):
        # Devuelve el estado de la electrovalvula
        return self.is_active


    def pad_listener(self):
        # Se suscribe al topic del mando para leer la pulsacion de las flechas y controlar la electrovalvula
        rospy.Subscriber('/robot/joy', Joy, self.callback)


    def callback(self, data):
        
        info_axes = data.axes
        
        # La cruceta (ARRIBA/ABAJO) se controla con el elemento 10 del info_axes
        # valor  1.0 -> ARRIBA. Activa la electrovalvula
        # valor -1.0 -> ABAJO. Desactiva la electrovalvula
        
        self.up_down_button = info_axes[10]

        if (self.up_down_button > 0.0 and not self.is_active):      # Pulsa ARRIBA y NO estaba activada antes -> Se activa
            rospy.logdebug("Pulsado: " + str(self.up_down_button))
            self.activate()   

        elif(self.up_down_button < 0.0 and self.is_active):         # Pulsa ABAJO y SI estaba activada antes -> Se desactiva
            rospy.loginfo("Pulsado: " + str(self.up_down_button))
            self.deactivate()



## ----------------------- MAIN --------------------------------------------------
if __name__ == "__main__":
    rospy.init_node('pad_electrovalve', anonymous=True, log_level=rospy.DEBUG)

    valve = Electrovalve()

    valve.pad_listener()
    rospy.spin()

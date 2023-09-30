#! /usr/bin/env python

""" ESTE ARCHIVO CONTIENE LA CLASE FruitHarvestingDemo CON TODAS LAS FUNCIONES PARA REALIZAR EL PROCESO """
""" Se importa en start_fruit_harvesting_demo.py para crear una instancia de la clase """
""" Tambien se puede ejecutar este .py directamente para hacer el procedimiento o pruebas de movimientos """

import rospy
import numpy as np
import tf
import actionlib

from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped, PoseWithCovarianceStamped, Twist

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseResult, MoveBaseGoal

from harvest_process.srv import RBKairosARM, RBKairosARMRequest
from electrovalve_control import Electrovalve


#Literal:
PI = np.pi
P1 = "robot"            # /robot -> REAL
P2 = "rbkairos"         # /rbkairos -> SIMULACION
NS_prefix = "/" + P1    # prefijo del NAMESPACE, para los topics y actions
FR_prefix = P1          # prefijo de FRAME, para los frames como "robot_map" etc.

FRAME = FR_prefix + '_map'



class FruitHarvestingDemo():
    def __init__(self, _coord, robot_approach_dist, arm_approach_dist, forward_dist = 0.1, backward_dist = 0.2, frame = FRAME, test_arm = False):
        ## _coord:              coordenadas del objeto
        ## robot_approach_dist: distancia entre el robot y el objeto para la navegacion autonoma de la base
        ## arm_approach_dist:   distancia entre la pinza y el objeto para el movimiento del brazo
        ## forward_dist:        distancia de avance de la pinza hacia el objeto para el movimiento del brazo
        ## backward_dist:       distancia de retroceso de la pinza tras agarrar el objeto para el movimiento del brazo
        ## frame:               sistema de referencia usado para indicar las coordenadas
        ## test_arm:            variable para activar o no el uso del brazo. Usado para pruebas
        
        # Publisher para visualizar en RVIZ la posicion objetivo de la base
        self.pub_base_goal = rospy.Publisher('/fruit_harvesting/base_goal_pose', PoseStamped, queue_size=1)

        # Cliente para mover la base con la navegacion autonoma
        self.move_base_client = actionlib.SimpleActionClient(NS_prefix + '/move_base', MoveBaseAction)

        # CONEXION CON EL SERVICIO DEL BRAZO
        self.test_arm = test_arm
        if self.test_arm:
            arm_service_name = '/fruit_harvesting/arm_service'
            rospy.wait_for_service(arm_service_name)
            self.move_arm_client = rospy.ServiceProxy(arm_service_name, RBKairosARM)

            # Se introducen los parametros en el mensaje de la peticion al servicio del brazo
            self.move_arm_request = RBKairosARMRequest()
            self.move_arm_request.object_coord = Point(*_coord)
            self.move_arm_request.approach_distance = arm_approach_dist
            self.move_arm_request.forward_distance = forward_dist
            self.move_arm_request.backward_distance = backward_dist

        # Se crean las variables que indicaran la posicion objetivo para el movimiento de la base
        self.base_goal = MoveBaseGoal()     # contiene posicion, orientacion y sistema de referencia
        self.pose = Pose()                  # contiene la posicion x,y,z
        self.ref_frame = frame              # sistema de referencia usado para los puntos

        # para terminar la ejecucion
        self._ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        # Se calcula el punto de aproximacion
        self.approach_point(_coord, robot_approach_dist)
        # self.approach_point_fake(_coord, robot_approach_dist)

        self.start_process()
        # self.prueba_colocacion()
        # self.execute_move_arm("GET_EE")
        # self.execute_move_arm("NAVIGATE")



    def pose_callback(self, pose_msg):
        rospy.logdebug("  in pose_callback")
        self.robot_position = pose_msg.pose.pose # posicion y orientacion
        rospy.logdebug("  in the pose_callback")
        # rospy.loginfo(self.robot_position)


    def get_robot_position(self):
        rospy.logdebug("  in get_robot_position")
        sub_pose = rospy.Subscriber(NS_prefix + '/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        rospy.sleep(1)


    def approach_point(self, obj_coord, gap_robot_obj):
        # Calculo del punto de aproximacion de la base movil. 
        # Depende de la separacion indicada en gap_robot_obj y del angulo que forma la linea imaginaria robot--objeto (para la orientacion)

        x_obj, y_obj = obj_coord[0:2]   # se usan solo las coordenadas x e y del objeto
        
        self.get_robot_position()
        x_robot = self.robot_position.position.x
        y_robot = self.robot_position.position.y
        rospy.loginfo("   X robot: %f, Y robot: %f", x_robot, y_robot)

        alpha_ = np.arctan2((y_robot-y_obj), (x_robot-x_obj))   # arctan2 devuelve el angulo del punto (Y,X) en el rango (-pi,pi)

        rospy.loginfo("   Angulo alpha: %f rad, %f grados", alpha_, alpha_*180/PI)

        x_app = x_obj + gap_robot_obj * np.cos(alpha_)
        y_app = y_obj + gap_robot_obj * np.sin(alpha_)

        self.approach_pt = (x_app, y_app, alpha_)


    def approach_point_fake(self, obj_coord, gap_robot_obj):
        # [FUNCION PARA PRUEBAS] para probar los resultados mandando siempre el mismo punto objetivo de aproximacion.
        # Basicamente, el punto de aproximacion se calcula en funcion del angulo alpha_, y ese angulo depende 
        # de la posicion relativa entre el objeto y el robot, asi que si el robot esta algo desplazado al lateral, el angulo alpha cambia y 
        # por tanto cambia el punto de aproximacion objetivo de la base. Aunque esto no deberia ser un problema para el brazo al coger la esfera, 
        # porque el objetivo del brazo se manda respecto al mapa, realmente el problema de colocacion del brazo se debe 
        # a la diferencia entre los puntos del mapa y los puntos reales del mundo real (causado por localizacion del robot).
        # Esta funcion es igual que approach_point, pero manteniendo la misma orientacion siempre, independientemente de alpha_.

        x_obj, y_obj = obj_coord[0:2]   # se usan solo las coordenadas x e y del objeto
        
        self.get_robot_position()
        x_robot = self.robot_position.position.x
        y_robot = self.robot_position.position.y
        rospy.loginfo("   X robot: %f, Y robot: %f", x_robot, y_robot)

        x_app = x_obj - gap_robot_obj 
        y_app = y_obj

        self.approach_pt = (x_app, y_app, PI)
    

    def go_approach(self):
        # Navegacion hacia la posicion y orientacion de aproximacion
        self.pose.position.x = self.approach_pt[0]
        self.pose.position.y = self.approach_pt[1]
        self.pose.position.z = 0

        alpha_ = self.approach_pt[2]
        rot_z = alpha_ - PI
        quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
        self.pose.orientation = Quaternion(*quat_aux)

        self.base_goal.target_pose.pose = self.pose
        self.base_goal.target_pose.header.frame_id = self.ref_frame
        
        reached_goal = False
        reached_goal = self.navigate_to_pose(self.base_goal, "APROXIMACION")

        return reached_goal
    

    def back_goal(self):
        # Navegacion hacia posicion de retirada
        self.get_robot_position()

        x_robot = self.robot_position.position.x
        y_robot = self.robot_position.position.y

        x_goal = x_robot - 1.5
        y_goal = y_robot + 0.3

        self.pose.position.x = x_goal
        self.pose.position.y = y_goal

        quat_aux = tf.transformations.quaternion_from_euler(0,0,-PI)
        self.pose.orientation = Quaternion(*quat_aux)

        self.base_goal.target_pose.pose = self.pose
        
        reached_goal = False
        reached_goal = self.navigate_to_pose(self.base_goal, "RETROCESO")

        return reached_goal
    

    def start_process(self, do_grab = True):
        # Se definen los pasos para realizar el procedimiento completo
        
        if do_grab:
            # Se crea una instancia de la clase Electrovalve para controlar su apertura y cierre
            EV = Electrovalve() 
        
        success = False
        # success = self.execute_move_arm("INIT")   # se coloca el brazo en una posicion cualquiera para despues mostrar el movimiento de plegado

        # Se pliega el brazo a la posicion de navegacion
        success = self.execute_move_arm("NAVIGATE")

        if success:
            # Se mueve la base hacia el punto de aproximacion
            success = self.go_approach()

            if success:
                # Se coloca el brazo en la posicion incial (INIT)
                success = self.execute_move_arm("INIT")

                if success:
                    # Se coloca el brazo en la posicion de acercamiento al objeto (APPROACH)
                    success = self.execute_move_arm("APPROACH")

                    if success:
                        # Se realiza el movimiento de agarre, moviendo la pinza hacia delante
                        success = self.execute_move_arm("PICK")

                        if success and do_grab:
                            # Se activa la electrovalvula para cerrar los dedos
                            success = EV.activate()

                            if success:
                                # Se realiza el movimiento de retroceso, moviendo la pinza hacia atras
                                success = self.execute_move_arm("BACK")

                                if success:
                                    # Se vuelve a plegar el brazo para la posterior navegacion
                                    success = self.execute_move_arm("NAVIGATE")

                                    if success:
                                        # Se mueve la base hacia el punto de retirada
                                        self.back_goal()

                        if success and do_grab:
                            # Se desactiva la electrovalvula para abrir los dedos
                            success = EV.deactivate()

                            
    def prueba_colocacion(self):
        # success = self.execute_move_arm("BACK")
        success = self.execute_move_arm("NAVIGATE")
        EV = Electrovalve()
        if success:
            success = self.go_approach()

            if success:
                success = self.execute_move_arm("INIT")

                if success:
                    success = self.execute_move_arm("APPROACH")

                    if success:
                        success = self.execute_move_arm("PICK")
                        rospy.sleep(3)
                        self.execute_move_arm("GET_EE")

                        if success:
                            # success = EV.activate()
                            if success:
                                # success = self.execute_move_arm("BACK")
                                # success = self.execute_move_arm("NAVIGATE")
                                rospy.sleep(3)
                                # EV.deactivate()


    def prueba_colocacion2(self):
        # success = self.execute_move_arm("BACK")
        # rospy.sleep(5)
        success = self.execute_move_arm("NAVIGATE")
        EV = Electrovalve()
        if success:
            # success = self.go_approach()

            if success:
                success = self.execute_move_arm("INIT")

                if success:
                    success = self.execute_move_arm("APPROACH")

                    if success:
                        success = self.execute_move_arm("PICK")

                        if success:
                            success = EV.activate()
                            if success:
                                success = self.execute_move_arm("BACK")
                                success = self.execute_move_arm("NAVIGATE")
                                # rospy.sleep()
                                self.back_goal()
                                EV.deactivate()


    def navigate_to_pose(self, goal_pose, pose_name = "aproximacion"):
        # Ejecuta la navegacion autonoma a la posicion indicada en 'goal_pose'.
        # 'pose_name' es el nombre para referirse al punto objetivo
        
        while self.pub_base_goal.get_num_connections()<1 and not self._ctrl_c:
            rospy.logwarn("Esperando conexion con publisher base_goal")
            rospy.sleep(0.5)

        goal_pose.target_pose.header.stamp = rospy.Time.now()
        self.pub_base_goal.publish(goal_pose.target_pose)
        rospy.logdebug(goal_pose.target_pose)        

        ## Navegacion hacia el punto indicado
        reached_goal = False
        while not(self._ctrl_c or reached_goal):
                
                self.move_base_client.wait_for_server()
                rospy.loginfo('Navegacion hacia el punto [{}]'.format(pose_name))
                self.move_base_client.send_goal(goal_pose, feedback_cb=self.move_base_callback)
                self.move_base_client.wait_for_result()
                result = self.move_base_client.get_state()

                rospy.logdebug("Resultado navegacion: {}".format(result))

                if result==3:
                    rospy.loginfo("  Punto [{}] alcanzado con exito!".format(pose_name))
                    reached_goal = True
                
                rospy.sleep(2)
        return reached_goal
        
    
    def execute_move_arm(self, motion_code):
        # Ejecuta el movimiento del brazo indicado en 'motion_code' haciendo una llamada al servicio del brazo

        if not self._ctrl_c and self.test_arm:
            self.move_arm_request.motion_code = motion_code

            move_arm_result = self.move_arm_client(self.move_arm_request)

            rospy.loginfo(move_arm_result.message)

            if not move_arm_result.move_successfull:
                rospy.logerr("  No se ha realizado el movimiento del brazo correctamente.")
                return
        
        return move_arm_result.move_successfull


    def move_base_callback(self, data):
        # print(data)
        return


    def shutdownhook(self):
        rospy.loginfo("shutdown time!")
        self._ctrl_c = True        
        self.move_base_client.cancel_all_goals()    # cancela los objetivos de la base y lo detiene




## -----------------------  MAIN  ----------------------------------------------
if __name__ == "__main__":
    rospy.init_node('prueba_base', log_level=rospy.DEBUG)
    move_obj = FruitHarvestingDemo([5.1,0,1.07], 1.05, 0.05, test_arm=True)


    prueba_EV = False
    if prueba_EV:
        EV = Electrovalve()
        EV.activate()
        rospy.sleep(1)
        EV.deactivate()

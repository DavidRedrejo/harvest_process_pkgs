#! /usr/bin/env python

""" ESTE ARCHIVO CONTIENE LA CLASE MoveArm CON TODAS LAS FUNCIONES PARA CONTROLAR EL BRAZO MANIPULADOR 
    CALCULA LOS PUNTOS DE LA TRAYECTORIA DEL BRAZO Y LAS POSICIONES ARTICULARES. 
    UTILIZA LAS FUNCIONES DE MOVEIT PARA MOVERLO (moveit_commander)"""
""" Se importa en move_arm_service_server.py para crear una instancia de la clase en el servidor del servicio """
""" Tambien se puede ejecutar este .py directamente para hacer pruebas del movimiento del brazo """

import rospy
import numpy as np
import tf, tf2_ros, tf2_geometry_msgs

import moveit_commander
import sys
import copy

from moveit_msgs.msg import MoveItErrorCodes, DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped, PoseWithCovarianceStamped
from harvest_process.srv import RBKairosARM, RBKairosARMResponse
from electrovalve_control import Electrovalve

#Literal:
PI = np.pi
P1 = "robot"       # /robot -> REAL
P2 = "rbkairos"    # /rbkairos -> SIMULACION
NS_prefix = "/" + P1
FR_prefix = P1
TEST_prefix = "/fruit_harvesting"

FRAME = FR_prefix + "_map"


class MoveArm ():
    def __init__(self, group = "arm", frame = FRAME, endeff = FR_prefix + "_soft_gripper_end_link", execution = False):

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group)

        self.group.set_end_effector_link(endeff)        # Se indica el sistema de referencia para el efector final
        self.group.set_pose_reference_frame(frame)      # Se indica el sistema de referencia para las coordenadas (mapa)

        # Se indican los valores maximos de velocidad y aceleracion para que el movimiento del brazo sea mas lento y seguro
        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)
        
        # Publishers para visualizar en RVIZ el punto del objeto y la posicion objetivo y la trayectoria
        self.pub_tool_goal = rospy.Publisher(TEST_prefix + '/tool_goal_pose', PoseStamped, queue_size=1)        # Posicion objetivo de la pinza
        self.pub_object_point = rospy.Publisher(TEST_prefix + '/object_point', PointStamped, queue_size=50)     # Punto en el que se encuentra el objeto
        self.pub_display_trajectory = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20)     # Trayectoria planificada

        # Cuando se termine el nodo se ejecutara la funcion shutdownhook, que por seguridad manda un mensaje de parada al brazo
        rospy.on_shutdown(self.shutdownhook)
        self._ctrl_c = False

        self.ref_frame = frame
        self._do_execution = execution

    
    def shutdownhook(self):
        rospy.loginfo("   ARM shutdown!!")
        self._ctrl_c = True
        self.group.stop()


    def set_joint_pose_case(self, case, do_execution = True, check_plan = False):
        # Mueve el brazo a las posiciones articulares definidas, segun el caso indicado en "case"
        # "do_execution" para ejecutar el movimiento
        # "check_plan" annade un retraso temporal para poder visualizar en RVIZ la planificacion del movimiento y comprobar que sea correcto

        """ orden de las articulaciones:
        - shoulder_pan
        - shoulder_lift
        - elbow
        - wrist_1
        - wrist_2
        - wrist_3 """
    
        if case.upper() == "INIT":
            # posicion "inicial" para posterior aproximacion al punto
            # joint_values = [1.05, -1.57, -2.45, 0.75, -0.25, 0.0]
            joint_values = [0.63, -1.57, -2.39, 0.88, 0.63, 0.0]

        elif case.upper() == "NAVIGATE":
            # posicion completamente plegada mientras navegacion de la base
            # joint_values = [0.88, -1.57, -2.85, 1.32, 1.88, 0.0]
            # joint_values = [0.88, -1.57, -2.85, 1.32, 2.19, 0.0]  # queda en el limite y la articulacion "choca", por la caja de colision (es mas grande de lo que realmente ocupa la pinza, por seguridad)
            joint_values = [0.88, -1.57, -2.85, 1.32, 2.01, 0.0]

        else:
            # para cualquier otro caso
            joint_values = [-0.44, -1.26, -2.51, 0.57, 1.88, 0.0]  # es mas seguro esto que ponerlo todo a 0

        self.group.set_joint_value_target(joint_values)
        plan = self.group.plan()

        rospy.logdebug("   [ARM]  Planificacion articular [{}] terminada".format(case))

        # Para ejecutar el movimiento planificado, se llama a la funcion "execute_plan"
        succes_move = self.execute_plan(plan, do_execution, check_plan, is_cartesian_path=False, name=case)
        return succes_move


    def get_ee_pose(self):
        # Devuelve la posicion del efector final, respecto a la base del brazo y respecto al mapa
        ee_pose = self.group.get_current_pose()
        rospy.loginfo(ee_pose)
        ee_pose_world = self.transform_pose(ee_pose.pose, FR_prefix + "_base_footprint", self.ref_frame)
        # ee_pose_world = ee_pose_worldStamped.pose
        rospy.loginfo(ee_pose_world)

        return [ee_pose, ee_pose_world]


    def pose_callback(self, pose_msg):
        self.robot_position = pose_msg.pose.pose # position and orientation
        rospy.logdebug("  in the pose_callback")
        # rospy.loginfo(self.robot_position)


    def get_robot_position(self):
        # Devuelve la posicion del robot (en la variable robot_position)
        sub_pose = rospy.Subscriber(NS_prefix + '/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.sleep(1)


    def approach_move(self, do_execution = False, check_plan = False): 
        # Obtencion del punto y orientacion para aproximar el brazo al objeto
        
        # NOTA: Primero se mueve el brazo a las coordenadas x_start,y_start (este punto de aproximacion calculado)
        # y despues de eso se hace un movimiento de avance en X visto desde el sistema del efector final.
        
        Xobj = self.obj_point.x             # x del objeto
        Yobj = self.obj_point.y             # y del objeto
        Zobj = self.obj_point.z             # z del objeto
        app_dist = self.approach_distance   # distancia de aproximacion al objeto

        # Posicion del robot
        self.get_robot_position()
        X_robot = self.robot_position.position.x
        Y_robot = self.robot_position.position.y
        rospy.loginfo("Posicion del robot\nX robot: %f, Y robot: %f", X_robot, Y_robot)

        # Angulo que forma el objeto (orientado como el mapa) respecto al robot
        alpha_ = np.arctan2((Y_robot-Yobj), (X_robot-Xobj))  # arctan2 devuelve el angulo del punto (Y,X) en el rango (-pi,pi)

        rospy.loginfo("Angulo alpha: %f rad, %f grados", alpha_, alpha_*180/PI)

        # Coordenadas (x,y) del primer punto de aproximacion
        x_start = Xobj + app_dist * np.cos(alpha_)
        y_start = Yobj + app_dist * np.sin(alpha_)

        # Creacion de la colocacion de aproximacion (posicion + orientacion)
        app_pose = Pose()
        app_pose.position = Point(x_start,y_start,Zobj)
        rot_z = alpha_ - PI
        quat_aux = tf.transformations.quaternion_from_euler(0,0,rot_z)
        app_pose.orientation = Quaternion(*quat_aux)

        # Publicacion de la posicion objetivo en RVIZ para visualizar
        pub_msg = PoseStamped()
        pub_msg.header.frame_id = self.ref_frame
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.pose = app_pose
        self.publish_(self.pub_tool_goal, pub_msg)  # publica para visualizar en RVIZ la posicion objetivo

        # Ejecucion de la trayectoria que pone la pinza en el punto de aproximacion
        self.group.set_pose_target(app_pose)  
        plan = self.group.plan()   # Planificacion de la trayectoria

        rospy.logdebug("   [ARM]  Planificacion del movimiento de aproximacion terminada")

        # Para ejecutar el movimiento planificado, se llama a la funcion "execute_plan"
        succes_move = self.execute_plan(plan, do_execution, check_plan, is_cartesian_path=False, name="APPROACH")
        return succes_move


    def transform_pose(self, input_pose, original_frame, new_frame):
        # Transforma la pose (posicion+orientacion) de entrada 'input_pose' desde el 'original_frame' hasta el 'new_frame'

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = original_frame
        pose_stamped.header.stamp = rospy.Time(0)
        # pose_stamped.header.stamp = rospy.Time.now()

        try:
            output_pose_stamped = tf_buffer.transform(pose_stamped, new_frame, rospy.Duration(1))
            output_pose = output_pose_stamped.pose
            return output_pose_stamped
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

  
    def pick_move(self, forward = 0.1, poses_number = 10, do_execution = False, check_plan = False):  
        # Se hace un movimiento de avance en la direccion X del efector final      
        # 'forward' indica el avance de la pinza en metros
        # 'poses_number' indica el numero de puntos de la trayectoria

        x_array = np.linspace(0, forward, poses_number)
        y_array = np.linspace(0, 0, poses_number)

        waypoints = []  # almacena todos los puntos que forman la trayectoria
        wpose = Pose()

        for x in x_array:
            wpose.position = Point(x,0,0)   # en este caso el movimiento es solo de avance en X

            quat_aux = tf.transformations.quaternion_from_euler(0,0,0)
            wpose.orientation = Quaternion(*quat_aux)

            # Se transforma la pose para cambiar desde el frame del efector final hasta el FRAME global (en este caso el mapa)
            wposeS_transformed = self.transform_pose(wpose, FR_prefix + "_soft_gripper_end_link", self.ref_frame)    
            wpose_transformed = wposeS_transformed.pose

            rospy.logdebug(wposeS_transformed)
            rospy.sleep(0.001)      # retraso para mostrar las posiciones objetivo una tras otra en RVIZ
            self.publish_(self.pub_tool_goal, wposeS_transformed)  # publica para visualizar en RVIZ la posicion objetivo

            waypoints.append(copy.deepcopy(wpose_transformed))      # se introduce la pose objetivo (transformada) al array de waypoints
        
        # Planificacion de la trayectoria completa con todos los puntos indicados en 'waypoints'
        ## eef_step = forward/poses_number
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints para seguir en la trayectoria
                                        forward/poses_number,        # eef_step: paso del efector final
                                        0.0)         # jump_threshold: salto permitido entre posiciones

        # Fraccion de la trayectoria seguida (1: completa, 0: no la sigue)
        rospy.loginfo("   fraccion de trayectoria seguida: {}".format(fraction))
        #rospy.logwarn("Posicon actual (INICIO) de las articualciones:")
        #rospy.logwarn(self.group.get_current_joint_values())

        # Se ejecuta el movimietno planificado
        succes_move = self.execute_plan(plan, do_execution, check_plan, is_cartesian_path=True, name="PICK")
        return succes_move


    def execute_plan(self, plan, do_execution, check_plan, is_cartesian_path = False, name = ""):
        # Funcion para ejecutar el movimiento planificado
        success_move = False
        
        if check_plan:      # Se anade un tiempo de espera para comprobar visualmente en RVIZ que la planificacion sea la correcta
            wait_time = 5   # Tiempo de espera en segundos
            rospy.logdebug("   [ARM]  CHECK PLAN -- {} segundos".format(wait_time))      
            rospy.sleep(wait_time)

        if do_execution and not self._ctrl_c:   # si se quiere hacer la planificacion Y NO SE HA CANCELADO, se ejecuta
            if not is_cartesian_path:
                success_move = self.group.go(wait=True)
            else:
                success_move = self.group.execute(plan, wait=True)

            if success_move:
                rospy.loginfo("   [ARM]  Movimiento [{}] TERMINADO".format(name))
            else:
                rospy.logwarn("   [ARM]  Movimiento [{}] NO COMPLETADO".format(name))

        return success_move


    def back_move(self, backward = 0.15, poses_number = 10, do_execution = False, check_plan = False):  
        # Se hace un movimiento de retroceso en la direccion X del efector final      
        # 'backward' indica el retroceso de la pinza en metros
        # 'poses_number' indica el numero de puntos de la trayectoria

        x_array = np.linspace(0, -backward, poses_number)
        y_array = np.linspace(0, 0, poses_number)

        waypoints = []  # almacena todos los puntos que forman la trayectoria
        wpose = Pose()

        for x in x_array:
            wpose.position = Point(x,0,0)   # en este caso el movimiento es solo en X

            quat_aux = tf.transformations.quaternion_from_euler(0,0,0)
            wpose.orientation = Quaternion(*quat_aux)

            # Se transforma la pose para cambiar desde el frame del efector final hasta el FRAME global (en este caso el mapa)
            wposeS_transformed = self.transform_pose(wpose, FR_prefix + "_soft_gripper_end_link", self.ref_frame)    
            wpose_transformed = wposeS_transformed.pose

            rospy.logdebug(wposeS_transformed)
            rospy.sleep(0.001)
            self.publish_(self.pub_tool_goal, wposeS_transformed)  # publica para visualizar en RVIZ la posicion objetivo

            waypoints.append(copy.deepcopy(wpose_transformed))      # se introduce la pose objetivo (transformada) al array de waypoints
        

        # Planificacion de la trayectoria completa con todos los puntos indicados en 'waypoints'
        ## eef_step = backward/poses_number
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints para seguir en la trayectoria
                                        backward/poses_number,        # eef_step: paso del efector final
                                        0.0)         # jump_threshold: salto permitido entre posiciones

        # Fraccion de la trayectoria seguida (1: completa, 0: no la sigue)
        rospy.loginfo("   fraccion de trayectoria seguida: {}".format(fraction))
        #rospy.logwarn("Posicon actual (INICIO) de las articualciones:")
        #rospy.logwarn(self.group.get_current_joint_values())

        succes_move = self.execute_plan(plan, do_execution, check_plan, is_cartesian_path=True, name="BACK")
        return succes_move


    # def pick_move_Z(self, forward = 0.1, poses_number = 10):  
    #     # Se hace un movimiento de avance en la direccion X del efector final      
    #     # 'forward' indica el avance de la pinza en metros
    #     # 'poses_number' indica el numero de puntos de la trayectoria

    #     z_array = np.linspace(0, forward, poses_number)
    #     y_array = np.linspace(0, 0, poses_number)

    #     waypoints = []  # almacena todos los puntos que forman la trayectoria
    #     wpose = Pose()

    #     for z in z_array:
    #         wpose.position = Point(0,0,z)   # en este caso el movimiento es solo de avance en X

    #         quat_aux = tf.transformations.quaternion_from_euler(0,0,0)
    #         wpose.orientation = Quaternion(*quat_aux)

    #         # Se transforma la pose para cambiar desde el frame del efector final hasta el FRAME global (en este caso el mapa)
    #         wposeS_transformed = self.transform_pose(wpose, FR_prefix + "_soft_gripper_end_link", self.ref_frame)    
    #         wpose_transformed = wposeS_transformed.pose

    #         rospy.logdebug(wposeS_transformed)
    #         rospy.sleep(0.001)
    #         self.publish_(self.pub_tool_goal, wposeS_transformed)  # publica para visualizar en RVIZ la posicion objetivo

    #         waypoints.append(copy.deepcopy(wpose_transformed))      # se introduce la pose objetivo (transformada) al array de waypoints
        

    #     # Planificacion de la trayectoria completa con todos los puntos indicados en 'waypoints'
    #     ## eef_step = forward/poses_number
    #     (plan, fraction) = self.group.compute_cartesian_path(
    #                                     waypoints,   # waypoints para seguir en la trayectoria
    #                                     forward/poses_number,        # eef_step: paso del efector final
    #                                     0.0)         # jump_threshold: salto permitido entre posiciones

    #     # Fraccion de la trayectoria seguida (1: completa, 0: no la sigue)
    #     rospy.loginfo("   fraccion de trayectoria seguida: {}".format(fraction))
    #     #rospy.logwarn("Posicon actual (INICIO) de las articualciones:")
    #     #rospy.logwarn(self.group.get_current_joint_values())

    #     # Ejecucion de la trayectoria planificada
    #     self.group.execute(plan, wait=True)

    #     rospy.loginfo("    [Movimiento PICK] terminado!")
       

    def set_point(self, point):
        # Indica el punto del objeto como atributo de la clase y publica para visualizar en RVIZ.
        # El parametro 'point' es del tipo geometry_msgs/Point
        self.obj_point = point

        point_msg = PointStamped()
        point_msg.point = point
        point_msg.header.frame_id = self.ref_frame
        point_msg.header.stamp = rospy.Time.now()

        self.publish_(self.pub_object_point, point_msg)


    def set_approach_distance(self, approach_distance):
        self.approach_distance = approach_distance


    def set_forward_backward_distance(self, fwd_dist, bwd_dist):
        self.forward_distance = fwd_dist
        self.backward_distance = bwd_dist

            
    def publish_(self, publisher, msg):
        # Esta funcion asegura que se publiquen los mensajes 
        # en el topico indicado. Hasta que no exista conexion no publica
        
        while publisher.get_num_connections() < 1 and not self._ctrl_c:
            rospy.sleep(0.5)
            rospy.logwarn("   Esperando conexion para publicar")
        
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)





## ------------------------  MAIN DEL MOVE_ARM (para pruebas)  -------------------------------
if __name__ == "__main__":
    rospy.init_node('prueba_brazo', log_level=rospy.DEBUG)
    test_move_arm = MoveArm()    
    
    test_move_arm.set_joint_pose_case("INIT")

    # prueba_EV = True    
    # test_move_arm.set_joint_pose_case("NAVIGATE")

    # punto = Point(5.2,0.2,1) # prueba de punto respecto a las coordenadas del mapa
    """
    test_move_arm.set_point(punto)
    test_move_arm.set_approach_distance(0.05) # 5 cm desde el centro de la bola hasta la punta de la pinza
    test_move_arm.approach_move(do_execution=True)

    test_move_arm.pick_move(do_execution=True)
    
    EV = Electrovalve()
    if prueba_EV:  
        EV.activate()
        # rospy.sleep(4)
        # EV.deactivate()

    test_move_arm.back_move(do_execution=False)

    EV.deactivate()
    """


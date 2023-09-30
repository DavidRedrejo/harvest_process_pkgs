#! /usr/bin/env python

""" ESTE ARCHIVO ES EL SERVIDOR DEL MOVIMIENTO DEL BRAZO
    Crea una instancia de la clase MoveArm y ejecuta las secuencias de movimiento segun la peticion que reciba el servicio """

import rospy

from harvest_process.srv import RBKairosARM, RBKairosARMResponse
from move_arm_class import MoveArm
from electrovalve_control import Electrovalve

#Literal:
P1 = "robot"       # /robot -> REAL
P2 = "rbkairos"    # /rbkairos -> SIMULACION
NS_prefix = "/" + P1
FR_prefix = P1
TEST_prefix = "/fruit_harvesting"

FRAME = FR_prefix + "_map"


def service_callback(request):
    rospy.logdebug("  [ARM SERVICE] service callback")
    move_arm.set_point(request.object_coord)
    move_arm.set_approach_distance(request.approach_distance)

    forward_dist = request.forward_distance
    backward_dist = request.backward_distance

    _check_plan = False  # True: anade un restraso en el movimiento del brazo para comprobar la planificacion desde RVIZ

    result = False

    response_msg = RBKairosARMResponse()

    if request.motion_code in ["INIT", "NAVIGATE"]:
        result = move_arm.set_joint_pose_case(request.motion_code, check_plan = _check_plan)

    elif request.motion_code == "APPROACH":     # [APPROACH]: posicion de acercamiento de la pinza al objeto
        result = move_arm.approach_move(do_execution = True, check_plan = _check_plan)
    
    elif request.motion_code == "PICK":         # [PICK]: movimiento de avance para agarrar el objeto
        result = move_arm.pick_move(forward=forward_dist, do_execution=True, check_plan=_check_plan)

    elif request.motion_code == "BACK":         # [BACK]: movimiento de retroceso para retirar el objeto
        result = move_arm.back_move(backward=backward_dist, do_execution=True, check_plan=_check_plan)

    elif request.motion_code == "mal":  # posicion de prueba
        result = move_arm.set_joint_pose_case("mal")

    elif request.motion_code == "GET_EE":       # devuelve la posicion del efector final
        move_arm.get_ee_pose()

    elif request.motion_code == "TEST":         # [TEST] sucuencia de movimientos de prueba
        result = move_arm.set_joint_pose_case("INIT")
        move_arm.approach_move(do_execution=True)       # lo coloca en el primer punto
        move_arm.pick_move(do_execution=True)           # hace el movimiento de avance en X

        if prueba_EV:  
            EV.activate()

        move_arm.back_move(do_execution=True)

        if prueba_EV:  
            EV.deactivate()

        response_msg.move_successfull = result
        response_msg.message = "Se ha realizado el movimiento correctamente." 

    elif request.motion_code == "TEST-2":  # [TEST-2] sucuencia de movimientos de prueba
        move_arm.set_joint_pose_case("INIT")
        move_arm.approach_move(do_execution=True)    # lo coloca en el primer punto
        move_arm.pick_move(do_execution=True)        # hace el movimiento de avance en X

        if prueba_EV:  
            EV.activate()

        if prueba_EV:  
            EV.deactivate()

        response_msg.move_successfull = True
        response_msg.message = "Se ha realizado el movimiento correctamente." 

    else:
        rospy.logerr("No se ha introducido un codigo valido [motion_code].")
        rospy.logwarn("[motion_code] valores posibles: 'INIT', 'NAVIGATE', 'APPROACH', 'PICK', 'BACK'.")
        response_msg.move_successfull = False
        response_msg.message = "No se ha introducido un codigo valido [motion_code]."


    if result:
        msg = "Movimiento [{}] TERMINADO CORRECTAMENTE".format(request.motion_code)
        response_msg.message = msg
        rospy.loginfo("   [ARM SERVICE]  " + msg)
    else:
        msg = "Movimiento [{}] NO COMPLETADO".format(request.motion_code)
        response_msg.message = msg
        rospy.logwarn("   [ARM SERVICE]  " + msg)
    
    response_msg.move_successfull = result   
    # rospy.loginfo("Motion code: [{}] \\ Exito: [{}]".format(request.motion_code, result))
    return response_msg


# --- MAIN ---

rospy.init_node('harvest_arm_service_server', log_level=rospy.DEBUG)

move_arm = MoveArm(frame=FRAME)

# Crea el servicio, usando el nombre '/fruit_harvesting/arm_service':
arm_service = rospy.Service('/fruit_harvesting/arm_service', RBKairosARM, service_callback)

prueba_EV = True
EV = Electrovalve()

rospy.spin() # mantiene el servicio en ejecucion

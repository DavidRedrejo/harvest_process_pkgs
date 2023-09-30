# harvest_process_pkgs

Conjunto de paquetes desarrollados para crear un demostrador de un proceso de recolección de fruta con una pinza modular blanda acoplada al robot RB-Kairos.

Este repositorio forma parte de un Trabajo Fin de Máster (TFM) en Ingeniería Industrial.

ROS version: kinetic. Ubuntu: 16.04 LTS

## Paquetes desarrollados
### soft_gripper_description
Crea la descripción de la herramienta (`soft_gripper.xacro`) y de la tabla con los componentes electroneumáticos (`components_table.xacro`) para poder añadirlas al robot.

### soft_gripper_moveit_config
Por medio del asistente (_MoveIt Setup Assistant_) se crea este paquete de configuración, para poder controlar y mover el brazo haciendo uso de las librerías y servicios de _MoveIt_.

### harvest_process
Crea las funciones necesarias para llevar a cabo la demostración del proceso.

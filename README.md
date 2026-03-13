# Xarm Lite6 Control 4.1

Repositorio para control y experimentación con el robot **xArm Lite6** usando **ROS2**.

Este proyecto permite ejecutar controladores **CTC** y **PID** y aplicar perturbaciones **senoidales** o **gaussianas** durante la ejecución de trayectorias.

---

## MIEMBROS
- Arturo Balboa
- Oscar de la Rosa
- Emiliano Niño
- Rigoberto Soto

# Requisitos

- Ubuntu 22.04
- ROS2 Humble
- Python 3
- xArm Lite6 driver

---

# Estructura del repositorio
Results/ → resultados y datos de experimentos
src/xarm_task/ → controlador principal del robot
src/xarm_perturbations/ → nodo para generar perturbaciones

---

# Ejecutar el sistema

El launch permite elegir:

- tipo de controlador
- tipo de perturbación

## CTC con perturbación senoidal
ros2 launch xarm_task Custom_launch.py controller_type:=CTC perturbation_mode:=sine

## CTC con perturbación gaussiana
ros2 launch xarm_task Custom_launch.py controller_type:=CTC perturbation_mode:=gaussian
## PID con perturbación senoidal
ros2 launch xarm_task Custom_launch.py controller_type:=PID perturbation_mode:=sine
## PID con perturbación gaussiana
ros2 launch xarm_task Custom_launch.py controller_type:=PID perturbation_mode:=gaussian

---

# Ejecutar sin perturbaciones
ros2 run xarm_task controller

Por defecto el código utiliza **CTC**.

Si se desea usar **PID sin perturbaciones**, modificar en:
src/xarm_task/xarm_task/controller.py

cambiando:
'CTC' por 'PID'

---

# Generar gráficas

Después de ejecutar las trayectorias, se pueden generar las gráficas con:
python3 Results/make_plots.py

---

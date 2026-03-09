Se hace uso de un launch para correr el codigo de perturbaciones y los de nuestro controllador, dependiendo de los argumentos es que podemos elegir entre CTC, PID y si queremos perturbaciones gausianas o senoidales
Comandos:
-ros2 launch xarm_task Custom_launch.py controller_type:=CTC perturbation_mode:=sine
-ros2 launch xarm_task Custom_launch.py controller_type:=CTC perturbation_mode:=gaussian
-ros2 launch xarm_task Custom_launch.py controller_type:=PID perturbation_mode:=sine
-ros2 launch xarm_task Custom_launch.py controller_type:=PID perturbation_mode:=gaussian

Si se quiere correr sin ninguna perturbaciones es el siguiente comando:
-ros2 run xarm_task controller
El comando anterior por defecto solo usa CTC, para poner PID sin perturbaciones, en el codigo de controller.py modifica la linea 41 'CTC' y la remplazas por 'PID'

Comando para generar graficas despues de haber realizado las trayectorias: python3 Results/make_plots.py

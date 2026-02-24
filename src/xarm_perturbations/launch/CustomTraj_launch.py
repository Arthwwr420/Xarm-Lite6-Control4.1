from launch import LaunchDescription
from launch_ros.actions import Node

#WRITE YOUR LAUNCH FILE HERE

def generate_launch_description():
    PID_node = Node(package= 'xarm_perturbations',
                    executable='position_controller',
                    output='screen'
                    )

    HEART_node = Node(package= 'xarm_perturbations',
                    executable='heart',
                    output='screen'
                    )


    ld = LaunchDescription([PID_node,HEART_node])
    return ld
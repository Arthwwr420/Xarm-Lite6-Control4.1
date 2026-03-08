from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#WRITE YOUR LAUNCH FILE HERE

def generate_launch_description():
    pp = LaunchConfiguration('pp')
    pi = LaunchConfiguration('pi')
    pd = LaunchConfiguration('pd')

    pp_arg = DeclareLaunchArgument('pp',default_value="[10,10,10]")
    pi_arg = DeclareLaunchArgument('pi',default_value="[0,0,0]")
    pd_arg = DeclareLaunchArgument('pd',default_value="[10,10,10]")

    kp_arg = [ LaunchConfiguration('pp') ]
    ki_arg = [ LaunchConfiguration('pp') ]
    kd_arg = [ LaunchConfiguration('pp') ]

    print(pp_arg)


    PID_node = Node(package= 'xarm_perturbations',
                    executable='position_controller',
                    output='screen',
                    parameters = [{
                        'kp': kp_arg,
                        'ki': ki_arg,
                        'kd': kd_arg,
                        'maxspeed':0.20,
                        'deadband':0.002,
                        'output_topic':"/servo_server/delta_twist_cmds",
                        }]
                    )


    HEART_node = Node(package= 'xarm_perturbations',
                    executable='heart',
                    output='screen'
                    )


    ld = LaunchDescription([
   			 PID_node,
			 HEART_node
			 ])
    return ld
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    controller_type = LaunchConfiguration('controller_type').perform(context).upper()
    perturbation_mode = LaunchConfiguration('perturbation_mode').perform(context).lower()

    controller_node = Node(
        package='xarm_task',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[{
            'controller_type': controller_type
        }]
    )

    if perturbation_mode == 'sine':
        perturbation_params = {
            'output_topic': '/servo_server/delta_joint_cmds',
            'pub_reliability': 'reliable',
            'enabled': True,
            'mode': 'sine',
            'sine_freq_hz': 1.0,
            'sine_amp_linear': 0.1,
            'base_linear': [0.02, -0.02, 0.02],
            'debug': True
        }

    elif perturbation_mode == 'gaussian':
        perturbation_params = {
            'output_topic': '/servo_server/delta_joint_cmds',
            'pub_reliability': 'reliable',
            'enabled': True,
            'mode': 'gaussian',
            'noise_std_linear': 0.1,
            'base_linear': [0.02, -0.02, 0.02]
        }

    else:
        raise ValueError(
            f"perturbation_mode invalido: '{perturbation_mode}'. Usa 'sine' o 'gaussian'."
        )

    perturbation_node = Node(
        package='xarm_perturbations',
        executable='perturbation_injector',
        name='perturbation_injector',
        output='screen',
        parameters=[perturbation_params]
    )

    return [
        controller_node,
        perturbation_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_type',
            default_value='PID',
            description='Controller type: PD, PID, or CTC'
        ),
        DeclareLaunchArgument(
            'perturbation_mode',
            default_value='sine',
            description='Perturbation mode: sine or gaussian'
        ),
        OpaqueFunction(function=launch_setup)
    ])
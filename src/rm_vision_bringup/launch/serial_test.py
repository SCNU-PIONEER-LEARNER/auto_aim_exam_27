import os
import sys
from yaml import safe_load
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def generate_launch_description():
    
    from common import launch_params, resolve_node_params_block
    from common import node_params as node_params_path
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace, SetParameter
    from launch.actions import TimerAction, Shutdown, ExecuteProcess, RegisterEventHandler
    from launch import LaunchDescription
    # from launch.event_handlers import OnProcessStart

    with open(node_params_path, 'r') as f:
        node_params = safe_load(f)
    namespace = launch_params.get('namespace', '')
    
    # for key in node_params:
    #     if 'ros__parameters' in node_params[key]:
    #         node_params[key]['ros__parameters']['use_sim_time'] = True

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='rm_serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[resolve_node_params_block(node_params, 'serial_driver', namespace)['ros__parameters']],
        ros_arguments=['--ros-args', '--log-level', 'INFO'],
    )

    virtual_low_board = Node(
        package='virtual_low_board',
        executable='virtual_low_board_node',
        name='virtual_low_board',
        output='both',
        emulate_tty=True,
        parameters=[{"data_mode": "smooth"}],
        ros_arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription([
        serial_driver_node,
        TimerAction(period=0.2 ,actions=[virtual_low_board])
    ])
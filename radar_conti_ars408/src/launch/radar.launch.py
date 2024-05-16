from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from pathlib import Path
from nav2_common.launch import RewrittenYaml


def launch_radar_node(context):
    can_channel = LaunchConfiguration('can_channel').perform(
        context)  # Here you'll get the runtime config value

    # Conditionally set can_channel in radar_node_param_overrides if it has a non-empty value
    radar_node_param_overrides = {}
    if can_channel:
        radar_node_param_overrides['can_channel'] = can_channel

    radar_node = Node(
        package='radar_conti_ars408',
        executable='radar_conti_ars408_composition',
        name='radar_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[LaunchConfiguration('params_file'), radar_node_param_overrides])

    return [radar_node]


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Full path to the ROS2 parameters file to use for the radar node'
    )
    can_channel_arg = DeclareLaunchArgument(
        'can_channel',
        default_value='',
        description='Declare the CAN channel '
    )

    lifecycle_nodes = ['radar_node']

    autostart = LaunchConfiguration('autostart')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                             default_value="False",
                                             description=str("Use sim time argument for whether to force it"))

    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='radar_lifecycle_manager',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    get_radar_node = OpaqueFunction(function=launch_radar_node)

    ld = LaunchDescription()
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(can_channel_arg)

    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(get_radar_node)

    return ld

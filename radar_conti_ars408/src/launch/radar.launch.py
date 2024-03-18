from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from pathlib import Path

def generate_launch_description():
    config_default_path = Path(get_package_share_directory('vayurobotics_config')) / 'config'
    print(config_default_path)
    
    lifecycle_nodes = ['conti_ars408_node']

    autostart = LaunchConfiguration('autostart')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    config_dir = LaunchConfiguration('config_dir')
    config_dir_arg = DeclareLaunchArgument('config_dir', default_value=str(config_default_path))

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
            name='conti_408_lifecycle_manager',
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    conti_ars408_node = Node(
            package='radar_conti_ars408',
            executable='radar_conti_ars408_composition',
            name='conti_ars408_node',
            namespace=namespace,
            output='screen',
            parameters=[{'can_channel': 'can0'}])

    ld = LaunchDescription()

    ld.add_action(config_dir_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)

    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(conti_ars408_node)

    return ld

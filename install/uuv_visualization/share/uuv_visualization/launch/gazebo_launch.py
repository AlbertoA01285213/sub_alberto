from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share_uuv_viz = get_package_share_directory('uuv_visualization')
    
    # --- 1. Definiciones de archivos ---
    urdf_file_path = os.path.join(pkg_share_uuv_viz, 'urdf', 'sub_cam.urdf')
    default_rviz_config_path = os.path.join(pkg_share_uuv_viz, 'rviz', 'gazebo_visualization.rviz')

    package_path = os.path.join(pkg_share_uuv_viz, '..')

    set_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        package_path
    )

    set_model_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_MODEL_PATH',
        package_path
    )

    with open(urdf_file_path, 'r') as f:
        robot_description_content = f.read()

    # Variables de entorno para solucionar el problema del "Black Screen"
    set_partition = SetEnvironmentVariable('IGN_PARTITION', 'uuv_sim')
    set_ip = SetEnvironmentVariable('IGN_IP', '127.0.0.1')

    set_plugin_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        os.path.join(get_package_share_directory('uuv_gazebo_plugins'), '../../lib/uuv_gazebo_plugins')
    )

    # --- 2. Simulación y Bridge ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            # launch_arguments={'gz_args': '-r empty.sdf'}.items()
            launch_arguments={'gz_args': '-r ' + os.path.join(pkg_share_uuv_viz, 'worlds', 'mundo_submarino.sdf')}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create', # Corregido: era executable
        arguments=[
            '-name', 'uuv',
            '-topic', 'robot_description',
            '-z', '-1.0'
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        additional_env={'IGN_PARTITION': 'uuv_sim'},
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/model/uuv/marker@ignition.msgs.Marker]ros_gz_bridge.msg.Marker',
            # Recibir la pose desde Gazebo
            # '/model/uuv/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
            # Enviar las fuerzas desde el ASMC hacia el Plugin
            # '/forces@std_msgs/msg/Float64MultiArray]ignition.msgs.Float_V',
            # '/model/uuv/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        # remappings=[
            # ('/model/uuv/pose', '/pose'),
            # ('/model/uuv/odometry', '/pose'),
        # ],
        output='screen'
    )
        
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Ruta absoluta al archivo de configuración de RViz2'
    )

    # --- 3. Nodos de Control y Navegación ---
    
    uuv_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True, # Cambiado a True para Gazebo
            'robot_description': robot_description_content,
        }]
    ) 

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='mission_2.yaml',
        description='Nombre del archivo yaml de la misión'
    )

    # Crear la ruta completa dinámicamente
    mission_path = [
        os.path.join(get_package_share_directory('uuv_mission'), 'missions', ''),
        LaunchConfiguration('mission_file')
    ]

    # --- En la definición del nodo mission_handler ---
    mission_handler = Node(
        package='uuv_mission',
        executable='mission_handler',
        name='mission_handler',
        parameters=[{
            'use_sim_time': True,
            'mission_file': mission_path  # <--- AQUÍ PASAMOS EL PARÁMETRO
        }]
    )
    
    trayectory_node = Node(
        package='uuv_navigation',
        executable='line_trayectory',
        name='line_trayectory',
        parameters=[{'use_sim_time': True}]
    )   

    los = Node(
        package='uuv_navigation',
        executable='los',
        name='los',
        parameters=[{'use_sim_time': True}]
    )  

    asmc = Node(
        package='uuv_control',
        executable='asmc',
        name='asmc',
        parameters=[{'use_sim_time': True}]
    )  

    dynamic_model = Node(
        package='uuv_control',
        executable='dynamic_model_uuv',
        name='dynamic_model',
        parameters=[{'use_sim_time': True}]
    )   

    # Nodo que asumo que publica TF o datos de telemetría adicionales
    uuv_pose_tracker_node = Node(
        package='uuv_visualization',
        executable='uuv_tracker',
        name='uuv_pose_tracker', # Cambié el nombre para no chocar con robot_state_publisher
        parameters=[{'use_sim_time': True}]
    )  

    object_visualizer_node = Node(
        package='uuv_visualization',
        executable='object_visualizer',
        name='object_visualizer',
        parameters=[{'use_sim_time': True}]
    )  
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}]
    )

    clean_zombies = ExecuteProcess(
        cmd=['pkill', '-9', 'ruby', ';', 'pkill', '-9', 'parameter_brid', ';', 'pkill', '-9', 'static_transform'],
        shell=True
    )

    # --- 4. Retornar la descripción (Asegúrate de incluir TODO aquí) ---
    return LaunchDescription([
        # clean_zombies,
        set_partition,
        set_ip,
        set_plugin_path,
        set_resource_path,
        set_model_path,
        rviz_config_arg,
        gazebo,
        spawn_robot, # Agregado
        bridge,      # Agregado
        uuv_state_publisher,
        mission_file_arg,
        mission_handler,
        trayectory_node,
        los,
        asmc,
        # dynamic_model,
        # uuv_pose_tracker_node,
        object_visualizer_node,
        # rviz_node,
    ])
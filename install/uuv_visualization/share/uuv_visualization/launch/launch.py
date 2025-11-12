from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Asumo que el paquete que contiene el launch file es 'kia_gd'
    pkg_share_kia_gd = get_package_share_directory('uuv_visualization')
    
    # --- 1. Definiciones de archivos ---
    
    # Ruta al URDF (ahora con 'base_link')
    urdf_file_path = os.path.join(pkg_share_kia_gd, 'urdf', 'sub.urdf')
    
    # Ruta al RViz config
    default_rviz_config_path = os.path.join(pkg_share_kia_gd, 'rviz', 'visualization_1.rviz')

    # Lee el contenido del URDF
    with open(urdf_file_path, 'r') as f:
        robot_description_content = f.read()
        
    # --- 2. Argumentos de Lanzamiento ---
    
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Ruta absoluta al archivo de configuración de RViz2'
    )

    # --- 3. Nodos ---

    # Publica la posición de la base del robot ('base_link') en el mundo ('map')
    # Este es el trabajo que PENSABAS que hacía robot_state_publisher.
    # El robot estará en (0,0,0) relativo a 'map'.
    # static_tf_map_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_base_link',
    #     arguments=['-0.35', '2.0', '-0.2', '0', '0', '1.57', 'map', 'base_link'] # 'map' -> 'base_link'
    # )

    # Lanza robot_state_publisher
    # Este nodo AHORA publicará 'base_link' -> 'visual_link'
    uuv_tracker = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='uuv_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_content,
            # No uses 'frame_prefix' ni 'namespace' a menos que
            # sepas exactamente por qué los necesitas. Causan confusión.
        }]
    ) 

    # bezier = Node(
    #     package='uuv_navigation',
    #     executable='bezier_trayectory',
    #     name='bezier_trayectory'
    # ) 

    mission_handler = Node(
        package='uuv_mission',
        executable='mission_handler',
        name='mission_handler'
    )
    
    bezier = Node(
        package='uuv_navigation',
        executable='line_trayectory',
        name='line_trayectory'
    )   

    los = Node(
        package='uuv_navigation',
        executable='los',
        name='los'
    )  

    pid = Node(
        package='uuv_control',
        executable='pid',
        name='pid'
    )   

    dynamic_model = Node(
        package='uuv_control',
        executable='dynamic_model_uuv',
        name='dynamic_model'
    )   

    uuv_pose_tracker = Node(
        package='uuv_visualization',
        executable='uuv_tracker',
        name='uuv_tracker'
    )  

    object_visualizer_node = Node(
        package='uuv_visualization',
        executable='object_visualizer',
        name='object_visualizer'
    )  
    
    # static_tf_map_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_base_link',
    #     arguments=['-0.35', '2.0', '-0.2', '0', '0', '1.57', 'map', 'base_link'] # 'map' -> 'base_link'
    # )

    # Lanza RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    # --- 4. Retornar la descripción ---
    return LaunchDescription([
        rviz_config_arg,
        uuv_tracker,
        mission_handler,
        bezier,
        los,
        pid,
        dynamic_model,
        uuv_pose_tracker,
        object_visualizer_node,
        rviz_node,
    ])
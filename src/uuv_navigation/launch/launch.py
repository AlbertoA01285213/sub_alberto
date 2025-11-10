import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'uuv_navigation' 
    executable_name = 'bezier.py' 
    node_name = 'trayectory_node' 
    param_file_name = 'bezier_params.yaml'
    db_file_name = 'waypoints.db'

    # --- 2. Obtener Rutas Absolutas ---
    
    # Ruta al directorio 'share' de tu paquete
    pkg_share_dir = get_package_share_directory(package_name)
    
    # Ruta al archivo de parámetros (asumiendo que está en 'config/')
    param_file = os.path.join(pkg_share_dir, 'config', param_file_name)
    
    # Ruta al archivo de la base de datos
    # Asumimos que guardas la DB en un directorio 'data/' dentro de 'share'
    db_file = os.path.join(pkg_share_dir, 'data', db_file_name)

    # --- 3. Definición del Nodo ---
    
    trayectory_node_executable = Node(
        package=package_name,
        executable=executable_name,
        name=node_name, # Nombre del nodo que buscará en el archivo YAML
        output='screen',
        emulate_tty=True,
        parameters=[
            # Carga todos los parámetros del archivo YAML
            param_file,
            {'db_path': '/home/alberto/Documents/sub_alberto/src/uuv_navigation/data/waypoints.db'}
        ]
    )
    
    # --- 4. Retornar la Descripción del Lanzamiento ---
    
    return LaunchDescription([
        trayectory_node_executable
        
        # Aquí también podrías añadir el nodo visualizador y RViz
        # para lanzar todo tu entorno de pruebas con un solo comando.
    ])
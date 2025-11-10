#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

class TrajectoryVisualizer(Node):

    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        # --- Parámetros ---
        # El topic donde tu robot publica la trayectoria
        self.declare_parameter('path_topic', '/robot_path') 
        # El topic donde se publicarán los marcadores para RViz
        self.declare_parameter('marker_topic', '/visualization_marker')
        
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value

        # --- Suscriptor ---
        # Se suscribe al topic de la trayectoria
        self.path_subscriber = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10)
        
        # --- Publicador ---
        # Publica los marcadores para RViz
        self.marker_publisher = self.create_publisher(Marker, marker_topic, 10)
        
        self.get_logger().info(f"Nodo visualizador de trayectoria iniciado.")
        self.get_logger().info(f"Escuchando trayectoria en: {path_topic}")
        self.get_logger().info(f"Publicando marcadores en: {marker_topic}")

    def path_callback(self, msg: Path):
        """
        Callback que se activa cada vez que se recibe un mensaje de Path.
        Convierte la trayectoria en un marcador de tipo POINTS.
        """
        
        if not msg.poses:
            self.get_logger().warn("Trayectoria recibida está vacía, no se publicarán marcadores.")
            return

        marker = Marker()
        
        # --- Cabecera (Header) ---
        # ¡Importante! El frame_id debe ser el mismo que el de la trayectoria
        marker.header.frame_id = msg.header.frame_id 
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # --- Identificación ---
        # Un 'namespace' y un 'id' únicos identifican al marcador.
        # Si publicas un nuevo marcador con el mismo ns/id, reemplazará al anterior.
        marker.ns = "trajectory_points"
        marker.id = 0
        
        # --- Tipo y Acción ---
        # POINTS: Muestra cada punto en la lista 'points' como un cubo o esfera.
        marker.type = Marker.POINTS
        # ADD: Añade/actualiza el marcador. DELETE borraría el marcador.
        marker.action = Marker.ADD
        
        # --- Pose ---
        # Para POINTS, la pose del marcador en sí es (0,0,0) y orientación (0,0,0,1)
        # Los puntos se definen relativos al frame_id del header.
        marker.pose.orientation.w = 1.0
        
        # --- Escala ---
        # Define el tamaño de cada esfera (en metros)
        marker.scale.x = 0.05  # 5 cm de diámetro
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # --- Color ---
        # Color verde brillante y totalmente opaco
        marker.color.g = 1.0  # Verde
        marker.color.a = 1.0  # Alfa (opacidad)
        
        # --- Lifetime ---
        # Cuánto tiempo persiste el marcador. Duration() (o 0s) significa "para siempre"
        # (o hasta que se reemplace).
        marker.lifetime = Duration()
        
        # --- Llenar los puntos ---
        # Extrae la posición (Point) de cada PoseStamped en la trayectoria
        marker.points = [pose_stamped.pose.position for pose_stamped in msg.poses]
        
        # Asignar un color individual a cada punto (opcional, pero útil)
        # Aquí los haremos de verde a rojo a lo largo de la trayectoria.
        marker.colors = []
        num_points = len(marker.points)
        for i in range(num_points):
            color = ColorRGBA()
            color.a = 1.0
            color.r = float(i) / num_points  # Incrementa el rojo
            color.g = 1.0 - (float(i) / num_points) # Decrementa el verde
            color.b = 0.0
            marker.colors.append(color)

        # --- Publicar ---
        self.marker_publisher.publish(marker)
        self.get_logger().info(f"Publicando {len(marker.points)} puntos de marcador.")


def main(args=None):
    rclpy.init(args=args)
    visualizer_node = TrajectoryVisualizer()
    try:
        rclpy.spin(visualizer_node)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
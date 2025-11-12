#!/usr/bin/env python3
import rclpy
import sqlite3
import os
import math
import random
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, DurabilityPolicy


class ObstaclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_publisher_node')
        
        # --- Parámetros ---
        self.declare_parameter('db_path', '/home/alberto/Documents/sub_alberto/src/uuv_visualization/data/obstacles.db')
        self.declare_parameter('marker_topic', '/obstacle_markers')
        self.declare_parameter('frame_id', 'world') # Frame al que están referidos los obstáculos

        # Obtener valores de parámetros
        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value
        marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # --- QoS "Latch" (Transient Local) ---
        # Esto asegura que el último mensaje publicado se guarde y
        # se envíe a cualquier suscriptor que se conecte tarde (como RViz).
        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- Publicador ---
        self.publisher = self.create_publisher(MarkerArray, marker_topic, latching_qos)
        
        # --- Lógica Principal ---
        # Usamos un temporizador de una sola vez para asegurarnos de que el
        # nodo esté completamente inicializado antes de publicar.
        self.timer = self.create_timer(0.5, self.load_and_publish_obstacles_once)

        self.get_logger().info("Nodo publicador de obstáculos iniciado.")
        self.get_logger().info(f"Publicando marcadores en: {marker_topic}")

    def load_and_publish_obstacles_once(self):
        # Cancelar el timer para que solo se ejecute una vez
        self.timer.cancel()

        if not os.path.exists(self.db_path):
            self.get_logger().fatal(f"¡Error! No se encuentra la base de datos: {self.db_path}")
            return
        
        self.get_logger().info(f"Cargando obstáculos desde: {self.db_path}")
        
        conn = None
        try:
            conn = sqlite3.connect(self.db_path)
            # Usar 'sqlite3.Row' permite acceder a las columnas por nombre
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            
            cursor.execute("SELECT * FROM obstacles")
            rows = cursor.fetchall()
            
            if not rows:
                self.get_logger().warn("La base de datos de obstáculos está vacía.")
                return

            marker_array = MarkerArray()
            current_time = self.get_clock().now().to_msg()
            
            for row in rows:
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = current_time
                
                # Namespace y ID únicos para cada marcador
                marker.ns = "static_obstacles"
                marker.id = row['id']
                
                # Tipo de forma
                marker.type = self.map_shape_to_type(row['shape'])
                marker.action = Marker.ADD
                
                # Posición
                marker.pose.position.x = row['pos_x']
                marker.pose.position.y = row['pos_y']
                marker.pose.position.z = row['pos_z']
                marker.pose.orientation.w = 1.0 # Orientación por defecto
                
                # Escala (tamaño)
                marker.scale.x = row['scale_x']
                marker.scale.y = row['scale_y']
                marker.scale.z = row['scale_z']
                
                # Color (Rojo semi-transparente)
                marker.color.r = (random.randint(1,1000))/1000
                marker.color.g = (random.randint(1,1000))/1000
                marker.color.b = (random.randint(1,1000))/1000
                marker.color.a = 0.6

                self.get_logger().info(f"¡Publicando marker!")
                
                # Lifetime (0 = para siempre)
                marker.lifetime = rclpy.duration.Duration().to_msg()
                
                marker_array.markers.append(marker)
                
            # Publicar el array completo
            self.publisher.publish(marker_array)
            self.get_logger().info(f"¡Publicados {len(marker_array.markers)} obstáculos estáticos!")

        except sqlite3.Error as e:
            self.get_logger().error(f"Error de SQLite al leer obstáculos: {e}")
        finally:
            if conn:
                conn.close()

    def map_shape_to_type(self, shape_str: str) -> int:
        """Convierte el string de la DB al tipo de marcador de RViz."""
        if shape_str == 'cube':
            return Marker.CUBE
        elif shape_str == 'sphere':
            return Marker.SPHERE
        elif shape_str == 'cylinder':
            return Marker.CYLINDER
        # Nota sobre pirámides (ver abajo)
        elif shape_str == 'arrow': # Usar 'arrow' como proxy simple de pirámide
            return Marker.ARROW
        else:
            self.get_logger().warn(f"Forma desconocida '{shape_str}', usando CUBE por defecto.")
            return Marker.CUBE

def main(args=None):
    rclpy.init(args=args)
    obstacle_publisher = ObstaclePublisher()
    try:
        rclpy.spin(obstacle_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
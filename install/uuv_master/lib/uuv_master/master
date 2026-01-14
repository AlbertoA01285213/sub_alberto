#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import sqlite3
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, String, Float32, Int16
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
from ament_index_python.packages import get_package_share_directory

class master(Node):
    def __init__(self):
        super().__init__('master')

        try:
            pkg_share = get_package_share_directory('uuv_vision')
            default_db_path = os.path.join(pkg_share, 'dada', 'obstacles.db')
        except Exception as e:
            self.get_logger().error(f"No se pudo encontrar el paquete: {e}")
            default_db_path = ""

        self.create_subscription(Int16, 'mission_status', self.mission_status_callback, 10)
        
        self.obstacle_pub = self.create_publisher(String, 'obstacle', 10)

        self.timer = self.create_timer(0.5, self.run)

    def mission_status_callback(self, msg: Int16):
        # 0 mission in proces
        # 1 mission complete
        self.mission_status = msg.data

    def run(self):
        # Solo actuamos si la misión anterior terminó
        if self.mission_status == 1:
            try:
                conn = sqlite3.connect(self.db_path)
                conn.row_factory = sqlite3.Row
                cursor = conn.cursor()

                # Buscamos el obstáculo con mayor prioridad que NO hayamos hecho
                # (Asumiendo que añadimos una columna 'completado' o simplemente por prioridad)
                cursor.execute("SELECT * FROM obstacles ORDER BY prioridad DESC LIMIT 1")
                row = cursor.fetchone()

                if row:
                    obstacle_name = row['nombre']
                    self.get_logger().info(f"Siguiente objetivo: {obstacle_name}")
                    
                    # Enviamos el nombre al mission_handler
                    msg = String()
                    msg.data = obstacle_name # Ej: "octagon"
                    self.obstacle_pub.publish(msg)
                    
                    # Cambiamos status a 0 para esperar a que el handler termine
                    self.mission_status = 0
                
                conn.close()
            except Exception as e:
                self.get_logger().error(f"Error en DB: {e}")








def main():
    rclpy.init()
    rclpy.spin(master())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
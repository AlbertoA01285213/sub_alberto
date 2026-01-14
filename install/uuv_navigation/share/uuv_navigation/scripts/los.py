#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool
import math

class LineOfSightNode(Node):

    def __init__(self):
        super().__init__('los_node')
        
        # --- Parámetros ---
        self.declare_parameter('path_topic', 'robot_path')
        self.declare_parameter('pose_topic', 'pose')
        self.declare_parameter('target_topic', 'pose_objetivo')
        self.declare_parameter('acceptance_radius', 0.5)

        # --- Obtener Parámetros ---
        path_topic = self.get_parameter('path_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        target_topic = self.get_parameter('target_topic').value
        self.acceptance_radius = self.get_parameter('acceptance_radius').value
        self.acceptance_radius_sq = self.acceptance_radius ** 2

        # --- Estado Interno ---
        self.current_path = None
        self.current_pose = None
        self.current_target_index = 0
        self.path_in_progress = False

        # --- Suscriptores y publicadores ---
        self.path_subscriber = self.create_subscription(Path, path_topic, self.path_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        self.target_publisher = self.create_publisher(Pose, target_topic, 10)
        self.checkpoint_publisher = self.create_publisher(Bool, 'checkpoint', 10)

        # --- Timer principal ---
        self.guidance_timer = self.create_timer(0.1, self.guidance_loop)
        
        self.get_logger().info("Nodo Line Of Sight iniciado.")
        self.get_logger().info(f"Trayectoria: {path_topic}")
        self.get_logger().info(f"Pose actual: {pose_topic}")
        self.get_logger().info(f"Objetivo publicado en: {target_topic}")

    def path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Se recibió un camino vacío.")
            self.current_path = None
            self.path_in_progress = False
            return
            
        self.current_path = msg
        self.current_target_index = 0
        self.path_in_progress = True
        # self.get_logger().info(f"Nuevo camino con {len(msg.poses)} waypoints.")

    def pose_callback(self, msg: PoseStamped):
        self.pose_actual_x = msg.pose.position.x
        self.pose_actual_y = msg.pose.position.y
        self.pose_actual_z = msg.pose.position.z
        
        roll, pitch, yaw = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        self.pose_actual_roll = roll
        self.pose_actual_pitch = pitch
        self.pose_actual_yaw = yaw
        self.current_pose = msg

    def reset_checkpoint(self):
        msg = Bool()
        msg.data = False
        self.checkpoint_publisher.publish(msg)

    def guidance_loop(self):
        if not self.path_in_progress or self.current_path is None or self.current_pose is None:
            return
        
        if self.current_target_index >= len(self.current_path.poses):
            self.finalizar_trayectoria()
            return

        # Si ya terminamos la lista de waypoints
        # if self.current_target_index >= len(self.current_path.poses):
        #     self.get_logger().info("¡Último waypoint alcanzado!")
        #     msg = Bool()
        #     msg.data = True
        #     self.checkpoint_publisher.publish(msg)
        #     self.create_timer(0.3, self.reset_checkpoint)
        #     self.path_in_progress = False
        #     self.current_path = None
        #     return

        # --- Obtener waypoint actual ---
        while self.current_target_index < len(self.current_path.poses):
            target_pose = self.current_path.poses[self.current_target_index].pose
            
            # Calcular distancia Euclidea (X, Y, Z)
            dx = self.pose_actual_x - target_pose.position.x
            dy = self.pose_actual_y - target_pose.position.y
            dz = self.pose_actual_z - target_pose.position.z
            dist_sq = dx**2 + dy**2 + dz**2

            # Calcular diferencia de YAW (para misiones de rotación)
            _, _, target_yaw = euler_from_quaternion([
                target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w
            ])
            yaw_error = abs(self.pose_actual_yaw - target_yaw)
            # Normalizar error de ángulo
            if yaw_error > math.pi: yaw_error = 2*math.pi - yaw_error

            # SI estamos cerca en posición Y en ángulo, pasamos al siguiente punto
            # (Si es solo rotación, dist_sq será 0, pero yaw_error mantendrá el punto activo)
            if dist_sq < self.acceptance_radius_sq and yaw_error < 0.15: # 0.15 rad approx 8 deg
                self.current_target_index += 1
            else:
                # Si el punto actual está lejos, este es nuestro objetivo
                break

        # self.get_logger().info(f"Distancia al waypoint {self.current_target_index}: {math.sqrt(dist_sq):.3f} m")

        # --- Verificar si alcanzamos el waypoint ---
        if self.current_target_index >= len(self.current_path.poses):
            self.finalizar_trayectoria()
        else:
            # 4. PUBLICAR SIEMPRE el objetivo actual
            current_target = self.current_path.poses[self.current_target_index].pose
            self.target_publisher.publish(current_target)

    def finalizar_trayectoria(self):
        """Función limpia para avisar que terminamos"""
        self.get_logger().info("🎯 Trayectoria completa. Enviando Checkpoint.")
        
        msg = Bool()
        msg.data = True
        self.checkpoint_publisher.publish(msg)
        
        # Resetear estado para no repetir el mensaje en el siguiente ciclo
        self.path_in_progress = False
        self.current_path = None
        self.current_target_index = 0

def main(args=None):
    rclpy.init(args=args)
    node = LineOfSightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

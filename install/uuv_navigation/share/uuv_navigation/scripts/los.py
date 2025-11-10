#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion
import math

class LineOfSightNode(Node):

    def __init__(self):
        super().__init__('los_node')
        
        # --- Parámetros ---
        self.declare_parameter('path_topic', '/robot_path')
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('target_topic', '/pose_objetivo')
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

        # --- Suscriptores y publicadores ---
        self.path_subscriber = self.create_subscription(Path, path_topic, self.path_callback, 10)
        self.pose_subscriber = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)
        self.target_publisher = self.create_publisher(Pose, target_topic, 10)

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
            return
            
        self.current_path = msg
        self.current_target_index = 0
        self.get_logger().info(f"Nuevo camino con {len(msg.poses)} waypoints.")

    def pose_callback(self, msg: Pose):
        self.pose_actual_x = msg.position.x
        self.pose_actual_y = msg.position.y
        self.pose_actual_z = msg.position.z
        
        roll, pitch, yaw = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        
        self.pose_actual_roll = roll
        self.pose_actual_pitch = pitch
        self.pose_actual_yaw = yaw

        self.current_pose = msg

    def guidance_loop(self):
        if self.current_path is None or self.current_pose is None:
            return

        if not self.current_path.poses:
            return

        # Si ya terminamos la lista de waypoints
        if self.current_target_index >= len(self.current_path.poses):
            final_target_pose = self.current_path.poses[-1].pose
            self.target_publisher.publish(final_target_pose)
            self.get_logger().info("Objetivo final alcanzado.")
            return

        target_pose = self.current_path.poses[self.current_target_index].pose
        
        # Distancia al waypoint actual
        dx = self.pose_actual_x - target_pose.position.x
        dy = self.pose_actual_y - target_pose.position.y
        dz = self.pose_actual_z - target_pose.position.z
        
        dist_sq = dx*dx + dy*dy + dz*dz

        # Si ya llegamos al waypoint
        if dist_sq < self.acceptance_radius_sq:
            self.get_logger().info(f"Waypoint {self.current_target_index} alcanzado.")
            self.current_target_index += 1

            # Si era el último
            if self.current_target_index >= len(self.current_path.poses):
                self.get_logger().info("¡Último waypoint alcanzado!")
                return
            
            target_pose = self.current_path.poses[self.current_target_index].pose
            self.get_logger().info(f"Nuevo objetivo: waypoint {self.current_target_index}")

        # Publicar target actual
        self.target_publisher.publish(target_pose)


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

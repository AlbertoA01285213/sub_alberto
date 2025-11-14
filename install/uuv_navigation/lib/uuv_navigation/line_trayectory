#!/usr/bin/env python3
import rclpy
import math
import os
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

class Trayectory_generator(Node):

    def __init__(self):
        super().__init__('trayectory_node')
        
        # Parámetros
        self.declare_parameter('path_topic', 'robot_path')
        self.declare_parameter('pose_topic', 'pose')
        self.declare_parameter('step_distance', 0.1)
        self.declare_parameter('db_path', '/home/alberto/Documents/sub_alberto/src/uuv_visualization/data/obstacles.db')

        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.step_distance = self.get_parameter('step_distance').get_parameter_value().double_value
        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value

        # Verificación DB (opcional)
        if not os.path.exists(self.db_path):
            self.get_logger().fatal(f"No se encuentra la DB: {self.db_path}")
            rclpy.shutdown()
            return

        # Estado actual
        self.current_pose = None
        self.new_waypoint_received = False

        # Último waypoint recibido
        self.waypoint = Pose()

        # ROS pubs/subs
        self.pose_subscriber = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)
        self.waypoint_subscriber = self.create_subscription(Pose, 'waypoint', self.waypoint_callback, 10)

        self.path_publisher = self.create_publisher(Path, path_topic, 10)
        self.marker_publisher = self.create_publisher(Marker, 'path_markers', 10)

        self.get_logger().info("Nodo generador de caminos iniciado.")
        self.get_logger().info(f"Escuchando pose en: {pose_topic}")
        self.get_logger().info(f"Publicando caminos en: {path_topic}")

    # ------------------ CALLBACKS ------------------

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def waypoint_callback(self, msg: Pose):
        # Comprobar si realmente cambió respecto al último waypoint
        dx = msg.position.x - self.waypoint.position.x
        dy = msg.position.y - self.waypoint.position.y
        dz = msg.position.z - self.waypoint.position.z

        # Diferencia de orientación
        q1 = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        q2 = [self.waypoint.orientation.x, self.waypoint.orientation.y, self.waypoint.orientation.z, self.waypoint.orientation.w]
        dq = sum((a - b)**2 for a, b in zip(q1, q2))

        # Umbrales de cambio mínimos (ajústalos según sensibilidad)
        pos_threshold = 1e-3     # metros
        ori_threshold = 1e-4     # diferencia en quaternion

        # if abs(dx) < pos_threshold and abs(dy) < pos_threshold and abs(dz) < pos_threshold and dq < ori_threshold:
        #     # El waypoint es esencialmente igual al anterior → ignorar
        #     return

        # Guardar el nuevo waypoint
        self.waypoint = msg
        self.new_waypoint_received = True

        # Generar nueva trayectoria solo si es un waypoint diferente
        self.get_logger().info("Nuevo waypoint detectado, generando trayectoria...")
        self.generate_path_callback()


    # ------------------ FUNCIONES ------------------

    def generate_path_callback(self):
        if self.current_pose is None:
            self.get_logger().warn("Aún no se ha recibido pose actual, no se puede generar trayectoria.")
            return
        if not self.new_waypoint_received:
            return

        # Marcar como procesado
        self.new_waypoint_received = False

        # Extraer posiciones
        x0, y0, z0 = self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z
        roll0, pitch0, yaw0 = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])

        x1, y1, z1 = self.waypoint.position.x, self.waypoint.position.y, self.waypoint.position.z
        roll1, pitch1, yaw1 = euler_from_quaternion([
            self.waypoint.orientation.x,
            self.waypoint.orientation.y,
            self.waypoint.orientation.z,
            self.waypoint.orientation.w
        ])

        # Calcular distancia y pasos
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        drx, dry, drz = roll1 - roll0, pitch1 - pitch0, yaw1 - yaw0
        dist = math.sqrt(dx*dx + dy*dy + dz*dz + drx*drx + dry*dry + drz*drz)
        steps = max(1, int(dist / self.step_distance))

        # Construir Path
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(steps + 1):
            t = i / float(steps)
            xi = x0 + dx * t
            yi = y0 + dy * t
            zi = z0 + dz * t
            rxi = roll0 + drx * t
            ryi = pitch0 + dry * t
            rzi = yaw0 + drz * t
            q = quaternion_from_euler(rxi, ryi, rzi)

            pose_s = PoseStamped()
            pose_s.header = path_msg.header
            pose_s.pose.position = Point(x=xi, y=yi, z=zi)
            pose_s.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path_msg.poses.append(pose_s)

        self.path_publisher.publish(path_msg)

        # También enviar markers para RViz
        marker = Marker()
        marker.header = path_msg.header
        marker.ns = "path_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 0.8
        marker.points = [pose.pose.position for pose in path_msg.poses]
        self.marker_publisher.publish(marker)

        # self.get_logger().info(f"Trayectoria publicada con {len(path_msg.poses)} puntos.")

# ------------------ MAIN ------------------
def main(args=None):
    rclpy.init(args=args)
    node = Trayectory_generator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

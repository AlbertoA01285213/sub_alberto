#!/usr/bin/env python3
import rclpy
import math
import os
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# --- Funciones utilitarias de quaterniones (sin dependencias externas) ---
def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)

def quat_dot(q1, q2):
    return q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]

def quat_slerp(q0, q1, t):
    """Spherical linear interpolation between q0 and q1 (each as (x,y,z,w))."""
    # normalize
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)

    dot = quat_dot(q0, q1)

    # If dot < 0, the quaternions have opposite handed-ness and slerp won't take
    # the shorter path. Fix by reversing one quaternion.
    if dot < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # Very close, use linear interpolation to avoid numerical problems
        x = q0[0] + t*(q1[0] - q0[0])
        y = q0[1] + t*(q1[1] - q0[1])
        z = q0[2] + t*(q1[2] - q0[2])
        w = q0[3] + t*(q1[3] - q0[3])
        return quat_normalize((x, y, z, w))

    # theta_0 = angle between input quaternions
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    x = (s0 * q0[0]) + (s1 * q1[0])
    y = (s0 * q0[1]) + (s1 * q1[1])
    z = (s0 * q0[2]) + (s1 * q1[2])
    w = (s0 * q0[3]) + (s1 * q1[3])
    return (x, y, z, w)


class Trayectory_generator(Node):

    def __init__(self):
        super().__init__('trayectory_node')
        
        # Parámetros
        self.declare_parameter('path_topic', 'robot_path')
        self.declare_parameter('pose_topic', 'pose')
        self.declare_parameter('step_distance', 0.1)
        self.declare_parameter('generation_interval', 0.5)
        self.declare_parameter('db_path', '/home/alberto/Documents/sub_alberto/src/uuv_visualization/data/obstacles.db')

        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.step_distance = self.get_parameter('step_distance').get_parameter_value().double_value
        interval = self.get_parameter('generation_interval').get_parameter_value().double_value
        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value

        # Verificación DB (si no te interesa, comenta)
        if not os.path.exists(self.db_path):
            self.get_logger().fatal(f"No se encuentra la DB: {self.db_path}")
            rclpy.shutdown()
            return

        # Estado: pose actual
        self.pose_actual_x = 0.0
        self.pose_actual_y = 0.0
        self.pose_actual_z = 0.0
        self.pose_actual_q = (0.0, 0.0, 0.0, 1.0)  # quaternion (x,y,z,w)

        # Estado: waypoint objetivo
        self.waypoint_x = 0.0
        self.waypoint_y = 0.0
        self.waypoint_z = 0.0
        self.waypoint_q = (0.0, 0.0, 0.0, 1.0)

        self.current_pose = None

        # Suscriptores y publicadores
        self.pose_subscriber = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)
        self.waypoint_subscriber = self.create_subscription(Pose, 'waypoint', self.waypoint_callback, 10)
        self.path_publisher = self.create_publisher(Path, path_topic, 10)
        self.timer = self.create_timer(interval, self.generate_path_callback)

        self.get_logger().info("Nodo generador de caminos iniciado.")
        self.get_logger().info(f"Escuchando pose en: {pose_topic}")
        self.get_logger().info(f"Publicando caminos en: {path_topic}")

    def waypoint_callback(self, msg: Pose):
        # Guardar posición objetivo
        self.waypoint_x = msg.position.x
        self.waypoint_y = msg.position.y
        self.waypoint_z = msg.position.z

        # Guardar quaternion objetivo si es válido; si viene (0,0,0,0) lo ignoramos
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        if q == (0.0, 0.0, 0.0, 0.0):
            self.get_logger().warn("Waypoint recibido con quaternion (0,0,0,0). Manteniendo orientación anterior.")
            return

        self.waypoint_q = quat_normalize(q)

    def pose_callback(self, msg: Pose):
        self.pose_actual_x = msg.position.x
        self.pose_actual_y = msg.position.y
        self.pose_actual_z = msg.position.z

        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # En caso de quaternion inválido, usar identidad
        if q == (0.0, 0.0, 0.0, 0.0):
            self.get_logger().warn("Pose actual con quaternion (0,0,0,0). Asignando identidad.")
            q = (0.0, 0.0, 0.0, 1.0)
        self.pose_actual_q = quat_normalize(q)

        self.current_pose = msg

    def generate_path_callback(self):
        if self.current_pose is None:
            # aún no hay pose actual
            return

        # Distancia posicional entre pose actual y waypoint
        dx = self.waypoint_x - self.pose_actual_x
        dy = self.waypoint_y - self.pose_actual_y
        dz = self.waypoint_z - self.pose_actual_z
        dist_pos = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Definir número de pasos solo según distancia posicional
        steps = max(1, int(dist_pos / self.step_distance))

        # Preparar mensaje Path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Interpolar posición + orientación (SLERP)
        for i in range(steps + 1):
            t = i / float(steps)
            xi = self.pose_actual_x + dx * t
            yi = self.pose_actual_y + dy * t
            zi = self.pose_actual_z + dz * t

            qi = quat_slerp(self.pose_actual_q, self.waypoint_q, t)
            # Normalizamos por seguridad
            qi = quat_normalize(qi)

            pose_s = PoseStamped()
            pose_s.header = path_msg.header
            pose_s.pose.position = Point(x=xi, y=yi, z=zi)
            pose_s.pose.orientation = Quaternion(x=qi[0], y=qi[1], z=qi[2], w=qi[3])

            path_msg.poses.append(pose_s)

        # Publicar el Path
        self.path_publisher.publish(path_msg)
        # logging opcional
        self.get_logger().debug(f"Publicado path con {len(path_msg.poses)} puntos (steps={steps}).")

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

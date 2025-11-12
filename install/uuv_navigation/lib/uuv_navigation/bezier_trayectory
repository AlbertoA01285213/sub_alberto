#!/usr/bin/env python3
import rclpy
import math
import sqlite3
import os
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


class Trayectory_generator(Node):
    """
    Generador de trayectorias mejorado:
      - Si la línea recta entre inicio y meta choca con un obstáculo (según muestreo),
        genera una curva cúbica de Bézier que lo esquive.
      - Publica Path con PoseStamped que incluyen orientación (yaw mirando al siguiente punto).
    """

    def __init__(self):
        super().__init__('trayectory_node')

        # parámetros existentes
        self.declare_parameter('path_topic', '/robot_path')
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('step_distance', 0.1)
        self.declare_parameter('generation_interval', 10.0)
        self.declare_parameter('db_path', '/home/alberto/Documents/sub_alberto/src/uuv_visualization/data/obstacles.db')

        # parámetros nuevos para evasión
        self.declare_parameter('obstacle_radius', 0.5)      # radio por defecto de los obstáculos en metros
        self.declare_parameter('safety_margin', 0.3)        # margen de seguridad extra
        self.declare_parameter('bezier_control_dist', 2.0)  # distancia base de los puntos de control desde la línea
        self.declare_parameter('line_check_step', 0.2)      # paso para muestreo en la comprobación de colisión

        # leer parámetros
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.step_distance = self.get_parameter('step_distance').get_parameter_value().double_value
        interval = self.get_parameter('generation_interval').get_parameter_value().double_value
        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value

        self.obstacle_radius = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        self.safety_margin = self.get_parameter('safety_margin').get_parameter_value().double_value
        self.bezier_control_dist = self.get_parameter('bezier_control_dist').get_parameter_value().double_value
        self.line_check_step = self.get_parameter('line_check_step').get_parameter_value().double_value

        # verificar DB
        if not os.path.exists(self.db_path):
            self.get_logger().fatal(f"No se encuentra la DB: {self.db_path}")
            rclpy.shutdown()
            return

        # estado
        self.pose_actual_x = 0.0
        self.pose_actual_y = 0.0
        self.pose_actual_z = 0.0
        self.pose_actual_roll = 0.0
        self.pose_actual_pitch = 0.0
        self.pose_actual_yaw = 0.0

        self.current_pose = None

        # subs/pubs
        self.pose_subscriber = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)
        self.path_publisher = self.create_publisher(Path, path_topic, 10)
        self.marker_pub = self.create_publisher(Marker, "/path_marker", 10)

        self.timer = self.create_timer(interval, self.generate_path_callback)

        self.get_logger().info("Nodo generador de caminos (con evasión) iniciado.")
        self.get_logger().info(f"Escuchando pose en: {pose_topic}")
        self.get_logger().info(f"Publicando caminos en: {path_topic}")

    # -----------------------
    # Callbacks y utilidades
    # -----------------------
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

    def publish_path_marker(self, waypoints):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # grosor de la línea

        # Color (azul por ejemplo)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.3
        marker.color.b = 1.0

        # llenar puntos
        for (x, y, z) in waypoints:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)

        self.marker_pub.publish(marker)    marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # grosor de la línea

        # Color (azul por ejemplo)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.3
        marker.color.b = 1.0

        # llenar puntos
        for (x, y, z) in waypoints:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)

        self.marker_pub.publish(marker)


    def get_random_waypoint_from_db(self):
        """Retorna (x,y,z) o None."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("SELECT pos_x, pos_y, pos_z FROM obstacles ORDER BY RANDOM() LIMIT 1")
            row = cursor.fetchone()
            return (float(row[0]), float(row[1]), float(row[2])) if row else None
        except Exception as e:
            self.get_logger().error(f"Error DB: {e}")
            return None
        finally:
            try:
                conn.close()
            except:
                pass

    def load_all_obstacles(self):
        """Carga todos los obstáculos de la BD como lista de tuples (x,y,z),
        excluyendo el waypoint actual guardado en self.r (si existe).
        Usa una tolerancia para comparación de floats.
        """
        obs = []
        tol = 1e-3  # tolerancia en metros para considerar "igual"
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("SELECT pos_x, pos_y, pos_z FROM obstacles")
            rows = cursor.fetchall()
            # Construir lista completa primero
            for row in rows:
                try:
                    ox = float(row[0])
                    oy = float(row[1])
                    oz = float(row[2])
                except Exception:
                    # si hay datos corruptos, saltar
                    continue
                obs.append((ox, oy, oz))
        except Exception as e:
            self.get_logger().error(f"Error cargando obstáculos: {e}")
        finally:
            try:
                conn.close()
            except Exception:
                pass

        # Si self.r (waypoint elegido) existe, filtrar obstáculos que coincidan con él
        try:
            # self.r debería ser (x,y,z) o None
            if hasattr(self, 'r') and self.r is not None:
                wx, wy, wz = float(self.r[0]), float(self.r[1]), float(self.r[2])
                filtered = []
                for (ox, oy, oz) in obs:
                    dx = ox - wx
                    dy = oy - wy
                    dz = oz - wz
                    d2 = dx*dx + dy*dy + dz*dz
                    if d2 <= (tol*tol):
                        # coincidencia (es el waypoint); lo ignoramos
                        continue
                    filtered.append((ox, oy, oz))
                obs = filtered
        except Exception:
            # en caso de que self.r no esté bien definida, devolvemos la lista completa
            pass

        return obs


    def point_distance_sq(self, x1, y1, z1, x2, y2, z2):
        dx = x1 - x2; dy = y1 - y2; dz = z1 - z2
        return dx*dx + dy*dy + dz*dz
    
    def is_relevant_obstacle(obstacle, waypoint, safe_margin):
        dx = obstacle.x - waypoint.x
        dy = obstacle.y - waypoint.y
        dz = obstacle.z - waypoint.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Si está muy cerca del waypoint, ignóralo
        return dist > safe_margin


    def line_collides_obstacles(self, x0, y0, z0, x1, y1, z1, obstacles, threshold):
        """
        Comprueba por muestreo si la línea recta entre (x0,y0,z0) y (x1,y1,z1) 
        está a distancia menor que threshold de algún obstáculo.
        threshold = obstacle_radius + safety_margin
        """
        dx = x1 - x0; dy = y1 - y0; dz = z1 - z0
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist == 0:
            return False, None  # sin colisión si mismo punto

        steps = max(2, int(dist / self.line_check_step))
        for i in range(steps + 1):
            t = i / steps
            sx = x0 + dx * t
            sy = y0 + dy * t
            sz = z0 + dz * t
            for (ox, oy, oz) in obstacles:
                d2 = self.point_distance_sq(sx, sy, sz, ox, oy, oz)
                if d2 <= threshold*threshold:
                    # retornar True y el obstáculo que provocó la colisión
                    return True, (ox, oy, oz)
        return False, None

    # -----------------------
    # Bézier helpers
    # -----------------------
    def cubic_bezier(self, p0, p1, p2, p3, t):
        """Evalúa curva cúbica de Bézier (p = tuple 3d) en t [0,1]."""
        u = 1.0 - t
        b0 = u*u*u
        b1 = 3*u*u*t
        b2 = 3*u*t*t
        b3 = t*t*t
        x = b0*p0[0] + b1*p1[0] + b2*p2[0] + b3*p3[0]
        y = b0*p0[1] + b1*p1[1] + b2*p2[1] + b3*p3[1]
        z = b0*p0[2] + b1*p1[2] + b2*p2[2] + b3*p3[2]
        return (x, y, z)

    def direction_to_quaternion(self, x1, y1, x2, y2):
        """Crea Quaternion (solo yaw) mirando de (x1,y1) a (x2,y2)."""
        yaw = math.atan2(y2 - y1, x2 - x1)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        q = Quaternion()
        q.w = cy
        q.x = 0.0
        q.y = 0.0
        q.z = sy
        return q

    def compute_bezier_around_obstacle(self, start, goal, obstacle):
        """
        Calcula 4 puntos de control (p0,p1,p2,p3) para una curva cúbica
        que evita 'obstacle' (ox,oy,oz).
        Estrategia:
         - proyectar todo en XY para definir desplazamiento perpendicular.
         - elegir lado (perp o -perp) con mayor clearance (comprobando distancia mínima a todos los obstáculos).
         - desplazar los puntos de control en XY hacia ese lado.
         - ajustar ligeramente el z (subir) si el obstáculo está cerca en z.
        """
        x0, y0, z0 = start
        x3, y3, z3 = goal
        ox, oy, oz = obstacle

        dx = x3 - x0; dy = y3 - y0; dz = z3 - z0
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist == 0:
            p0 = (x0, y0, z0); p3 = (x3, y3, z3)
            return p0, p0, p3, p3

        # dirección unitaria
        ux, uy, uz = dx/dist, dy/dist, dz/dist

        # vector perpendicular en XY
        perp_x = -uy
        perp_y = ux
        perp_len = math.sqrt(perp_x*perp_x + perp_y*perp_y)
        if perp_len == 0:
            perp_x, perp_y = 0.0, 0.0
        else:
            perp_x /= perp_len; perp_y /= perp_len

        # magnitude de desplazamiento: función de radio obstáculo + margen + control_dist
        offset_mag = (self.obstacle_radius + self.safety_margin) + self.bezier_control_dist

        # probar ambos lados y elegir el mejor según clearance a todos los obstáculos
        side_candidates = [1.0, -1.0]
        best_side = 1.0
        best_min_dist = -1.0
        all_obstacles = self.load_all_obstacles()
        for s in side_candidates:
            # construir control points candidatos
            p1_xy = (x0 + ux * (dist * 0.25) + perp_x * offset_mag * s,
                     y0 + uy * (dist * 0.25) + perp_y * offset_mag * s)
            p2_xy = (x0 + ux * (dist * 0.75) + perp_x * offset_mag * s,
                     y0 + uy * (dist * 0.75) + perp_y * offset_mag * s)
            # medir distancia mínima de estos dos puntos a todos los obstáculos (xy)
            min_d = float('inf')
            for (ax, ay, az) in all_obstacles:
                dx1 = p1_xy[0] - ax; dy1 = p1_xy[1] - ay
                dx2 = p2_xy[0] - ax; dy2 = p2_xy[1] - ay
                dmin = min(math.hypot(dx1, dy1), math.hypot(dx2, dy2))
                if dmin < min_d:
                    min_d = dmin
            if min_d > best_min_dist:
                best_min_dist = min_d
                best_side = s

        # construir p0..p3 incluyendo z: elevamos z si obstáculo tiene z similar
        # control points XY con lado elegido
        p0 = (x0, y0, z0)
        p3 = (x3, y3, z3)

        p1 = (x0 + ux * (dist * 0.25) + perp_x * offset_mag * best_side,
              y0 + uy * (dist * 0.25) + perp_y * offset_mag * best_side,
              z0 + max(0.0, (oz - z0) * 0.5))  # ligera elevación si obstáculo está por encima de start

        p2 = (x0 + ux * (dist * 0.75) + perp_x * offset_mag * best_side,
              y0 + uy * (dist * 0.75) + perp_y * offset_mag * best_side,
              z0 + max(0.0, (oz - z0) * 0.25))  # menor elevación hacia objetivo

        return p0, p1, p2, p3

    # -----------------------
    # Generación de camino
    # -----------------------
    def generate_path_callback(self):
        if self.current_pose is None:
            self.get_logger().warn("Aún no se ha recibido pose, esperando...")
            return

        self.r = self.get_random_waypoint_from_db()
        if self.r is None:
            self.get_logger().warn("No hay waypoint disponible en la DB.")
            return
        goal_point = Point(x=self.r[0], y=self.r[1], z=self.r[2])

        start = (self.pose_actual_x, self.pose_actual_y, self.pose_actual_z)
        goal = (goal_point.x, goal_point.y, goal_point.z)

        self.get_logger().info(
            f"Generando camino desde ({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f}) "
            f"hacia ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f})"
        )

        # carga obstáculos y verifica colisión con línea recta
        obstacles = self.load_all_obstacles()
        # safe_margin = 1.0  # puedes ponerlo como parámetro si quieres
        # filtered_obstacles = []
        # for (ox, oy, oz) in obstacles:
        #     dx = ox - goal[0]
        #     dy = oy - goal[1]
        #     dz = oz - goal[2]
        #     dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        #     # ignora el waypoint objetivo
        #     if dist < safe_margin:
        #         continue  

        #     filtered_obstacles.append((ox, oy, oz))

        # obstacles = filtered_obstacles
        threshold = self.obstacle_radius + self.safety_margin
        collides, closest_obstacle = self.line_collides_obstacles(
            start[0], start[1], start[2], goal[0], goal[1], goal[2], obstacles, threshold
        )

        waypoints = []
        if not collides:
            # línea recta: sample points
            dx = goal[0] - start[0]; dy = goal[1] - start[1]; dz = goal[2] - start[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist == 0:
                waypoints = [(start[0], start[1], start[2])]
            else:
                steps = max(1, int(dist / self.step_distance))
                for i in range(steps):
                    t = i / steps
                    waypoints.append((start[0] + dx * t, start[1] + dy * t, start[2] + dz * t))
                waypoints.append(goal)
            self.get_logger().info("Trayectoria recta - sin colisión detectada.")
        else:
            # generar Bézier alrededor del obstáculo detectado
            self.get_logger().info(f"Colisión detectada con obstáculo en {closest_obstacle}, generando Bézier.")
            p0, p1, p2, p3 = self.compute_bezier_around_obstacle(start, goal, closest_obstacle)
            # samplear la bézier
            dx = goal[0] - start[0]; dy = goal[1] - start[1]; dz = goal[2] - start[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            steps = max(4, int(dist / self.step_distance) * 2)  # muestreo algo más fino en curva
            for i in range(steps):
                t = i / float(steps)
                waypoints.append(self.cubic_bezier(p0, p1, p2, p3, t))
            waypoints.append(p3)
            self.get_logger().info(f"Bézier generada con {len(waypoints)} puntos.")

        # construir Path con orientación hacia el siguiente punto
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(waypoints)):
            x, y, z = waypoints[i]
            pose_s = PoseStamped()
            pose_s.header = path_msg.header
            pose_s.pose.position.x = x
            pose_s.pose.position.y = y
            pose_s.pose.position.z = z

            # orientación: mirar al siguiente punto (si existe), si no usar anterior
            if i < len(waypoints) - 1:
                x2, y2, z2 = waypoints[i+1]
            elif i > 0:
                x2, y2, z2 = waypoints[i-1]
            else:
                x2, y2, z2 = (x + 1.0, y, z)

            pose_s.pose.orientation = self.direction_to_quaternion(x, y, x2, y2)
            path_msg.poses.append(pose_s)

        # publicar
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"Camino publicado con {len(path_msg.poses)} puntos.")
        self.publish_path_marker(waypoints)


def main(args=None):
    rclpy.init(args=args)
    node = Trayectory_generator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

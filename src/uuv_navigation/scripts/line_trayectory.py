#!/usr/bin/env python3
import rclpy
import math
import sqlite3
import os
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf_transformations import euler_from_quaternion

class Trayectory_generator(Node):

    def __init__(self):
        super().__init__('trayectory_node')
        
        self.declare_parameter('path_topic', 'robot_path')
        self.declare_parameter('pose_topic', 'pose')
        self.declare_parameter('step_distance', 0.1)
        self.declare_parameter('generation_interval', 10.0)
        self.declare_parameter('db_path', '/home/alberto/Documents/sub_alberto/src/uuv_visualization/data/obstacles.db')

        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.step_distance = self.get_parameter('step_distance').get_parameter_value().double_value
        interval = self.get_parameter('generation_interval').get_parameter_value().double_value
        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value

        # Verificación DB
        if not os.path.exists(self.db_path):
            self.get_logger().fatal(f"No se encuentra la DB: {self.db_path}")
            rclpy.shutdown()
            return

        # Pose actual numérica
        self.pose_actual_x = 0.0
        self.pose_actual_y = 0.0
        self.pose_actual_z = 0.0
        self.pose_actual_roll = 0.0
        self.pose_actual_pitch = 0.0
        self.pose_actual_yaw = 0.0

        self.waypoint_x = 0.0
        self.waypoint_y = 0.0
        self.waypoint_z = 0.0

        self.current_pose = None

        self.pose_subscriber = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)
        self.waypoint_subscriber = self.create_subscription(Pose, 'waypoint', self.waypoint_callback, 10)
        
        self.path_publisher = self.create_publisher(Path, path_topic, 10)
        self.timer = self.create_timer(interval, self.generate_path_callback)

        self.get_logger().info(f"Nodo generador de caminos iniciado.")
        self.get_logger().info(f"Escuchando pose en: {pose_topic}")
        self.get_logger().info(f"Publicando caminos en: {path_topic}")

    def waypoint_callback(self, msg: Pose):
        self.waypoint_x = msg.position.x
        self.waypoint_y = msg.position.y
        self.waypoint_z = msg.position.z

        roll, pitch, yaw = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        self.waypoint_roll = roll
        self.waypoint_pitch = pitch
        self.waypoint_yaw = yaw

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


    def generate_path_callback(self):
        if self.current_pose is None:
            self.get_logger().warn("Aún no se ha recibido pose, esperando...")
            return
        
        goal_point = Point(x=self.waypoint_x, y=self.waypoint_y, z=self.waypoint_z)

        self.get_logger().info(
            f"Generando camino desde ({self.pose_actual_x:.2f}, {self.pose_actual_y:.2f}, {self.pose_actual_z:.2f}) "
            f"hacia ({goal_point.x:.2f}, {goal_point.y:.2f}, {goal_point.z:.2f})"
        )

        waypoints = self.interpolate_line(
            self.pose_actual_x, self.pose_actual_y, self.pose_actual_z,
            goal_point,
            self.step_distance
        )

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(waypoints)):
            x, y, z =  waypoints[i]
            pose_s = PoseStamped()
            pose_s.header = path_msg.header
            pose_s.pose.position.x = x
            pose_s.pose.position.y = y
            pose_s.pose.position.z = z
            path_msg.poses.append(pose_s)
            

        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"Camino publicado con {len(waypoints)} puntos.")

    def interpolate_line(self, x0, y0, z0, goal_point, step):
        waypoints = []

        dx = goal_point.x - x0
        dy = goal_point.y - y0
        dz = goal_point.z - z0

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist == 0:
            return [(x0, y0, z0)]

        steps = int(dist / step)
        ux, uy, uz = dx/dist, dy/dist, dz/dist

        for i in range(steps):
            waypoints.append((x0 + ux*i*step, y0 + uy*i*step, z0 + uz*i*step))

        waypoints.append((goal_point.x, goal_point.y, goal_point.z))
        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = Trayectory_generator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

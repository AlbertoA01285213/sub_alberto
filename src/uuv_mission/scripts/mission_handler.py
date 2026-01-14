#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, String, Float32, Int16
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
from ament_index_python.packages import get_package_share_directory

class MissionHandler(Node):
    def __init__(self):
        super().__init__('mission_handler')

        try:
            default_path = os.path.join(
                get_package_share_directory('uuv_mission'), 
                'missions', 'survey.yaml'
            )
        except:
            default_path = ""

        self.declare_parameter('mission_file', default_path)
        mission_path = self.get_parameter('mission_file').value

        self.get_logger().info(f"Cargando misión desde: {mission_path}")

        # 3. Validar si el archivo existe antes de abrirlo
        if not os.path.exists(mission_path):
            self.get_logger().error(f"¡ARCHIVO DE MISIÓN NO ENCONTRADO!: {mission_path}")
            return

        with open(mission_path, 'r') as f:
            self.mission = yaml.safe_load(f)

        self.actions = self.mission["actions"]
        self.idx = 0
        self.state = "RUNNING"
        self.pose_actual = [0]*6
        self.checkpoint = 0  # ✅ inicializado

        self.create_subscription(Bool, 'checkpoint', self.checkpoint_callback, 10)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.mission_sub = self.create_subscription(String, 'load_mission', self.load_mission_callback, 10)
        self.wp_pub = self.create_publisher(PoseStamped, 'waypoint', 10)
        self.checkpoint_pub = self.create_publisher(Bool, 'checkpoint', 10)
        self.status_pub = self.create_publisher(Int16, 'mission_status', 10)
        self.picture_pub = self.create_publisher(Int16, 'take_picture', 10)

        self.timer = self.create_timer(0.5, self.run)

    def checkpoint_callback(self, msg: Bool):
        self.checkpoint = msg.data  # ✅ corregido

    def pose_callback(self, msg: PoseStamped):
        self.pose_actual[0] = msg.pose.position.x
        self.pose_actual[1] = msg.pose.position.y
        self.pose_actual[2] = msg.pose.position.z
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.pose_actual[3] = roll
        self.pose_actual[4] = pitch
        self.pose_actual[5] = yaw

    def load_mission_callback(self, msg: String):
        mission_name = msg.data
        pkg_share = get_package_share_directory('uuv_mission')
        new_path = os.path.join(pkg_share, 'missions', f"{mission_name}.yaml")

        if os.path.exists(new_path):
            with open(new_path, 'r') as f:
                self.mission = yaml.safe_load(f)
                self.actions = self.mission["actions"]
                self.idx = 0  # Reiniciamos la misión
                self.get_logger().info(f"Nueva misión cargada: {mission_name}")
                
                # Publicamos que estamos trabajando (status 0)
                status_msg = Int16()
                status_msg.data = 0
                self.status_pub.publish(status_msg)
        else:
            self.get_logger().error(f"No existe el archivo: {new_path}")

    def run(self):
        if self.idx >= len(self.actions):
            # Publicar status solo una vez cuando termina
            if not hasattr(self, 'mission_finished_flag'):
                self.get_logger().info("¡MISIÓN COMPLETADA!")
                status_msg = Int16()
                status_msg.data = 1
                self.status_pub.publish(status_msg)
                self.mission_finished_flag = True
            return

        action = self.actions[self.idx]

        if not hasattr(self, 'current_idx_logged') or self.current_idx_logged != self.idx:
            self.get_logger().info(f"Ejecutando Acción {self.idx}: {action['type']}")
            self.current_idx_logged = self.idx
            # IMPORTANTE: Si es un movimiento nuevo, ignoramos checkpoints viejos
            if action["type"] in ["goto", "rotate"]:
                self.checkpoint = 0
                

        if action["type"] == "goto":
            wp = action["waypoint"]
            msg = PoseStamped()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"

            msg.pose.position.x = float(wp[0])
            msg.pose.position.y = float(wp[1])
            msg.pose.position.z = float(wp[2])

            q = quaternion_from_euler(wp[3], wp[4], wp[5])
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]

            self.wp_pub.publish(msg)
            self.get_logger().info(f"Waypoint {wp[0]}, {wp[1]}, {wp[2]}, {wp[3]}, {wp[4]}, {wp[5]}")

            if self.checkpoint == 1:
                self.checkpoint = 0
                self.idx += 1
                


        elif action["type"] == "hold":
            duration = action["duration"]
            if not hasattr(self, "hold_start"):
                # self.get_logger().info(f"Holding for {duration} seconds")
                self.hold_start = time.perf_counter()
                # self.get_logger().info()(f"Hold {duration}")

            if time.perf_counter() - self.hold_start >= duration:
                del self.hold_start
                self.idx += 1


        elif action["type"] == "publish":
            information = action["message"]
            msg_data_str = information[0]   # Esto es el string 'True'
            topic_name_str = information[1] # Esto es el string 'checkpoint'

            # Comparamos el string de Python con el string que esperamos
            if topic_name_str == 'checkpoint':
                # Creamos un mensaje Bool vacío
                msg_to_publish = Bool()
                # Asignamos el valor booleano a su atributo .data
                msg_to_publish.data = (msg_data_str.lower() == 'true') # Convierte 'True' a True

                # Publicamos el mensaje
                self.checkpoint_pub.publish(msg_to_publish)
                self.idx +=1
            else:
                self.get_logger().warn(f"Acción 'publish' para un tópico no implementado: {topic_name_str}")
                self.idx += 1 # Avanzamos para no bloquear la misión


        elif action["type"] == "rotate":
            rotations = action["rotation"]

            qx, qy, qz, qw = quaternion_from_euler(
                self.pose_actual[3] + rotations[0],
                self.pose_actual[4] + rotations[1],
                self.pose_actual[5] + rotations[2]
            )

            msg = PoseStamped()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            
            msg.pose.position.x = self.pose_actual[0]
            msg.pose.position.y = self.pose_actual[1]
            msg.pose.position.z = self.pose_actual[2]
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw

            self.wp_pub.publish(msg)

            if self.checkpoint == 1:
                self.checkpoint = 0
                self.idx += 1
        
        elif action["type"] == "picture":
            take_picture_msg = Int16()
            take_picture_msg.data = 1
            self.picture_pub.publish(take_picture_msg)
            
            # Es importante avanzar el índice para que pase a la siguiente acción
            self.idx += 1 
            self.get_logger().info("Foto disparada")

        if self.idx>= len(self.actions):
            status_msg = Int16()
            status_msg.data = 1
            self.status_pub.publish(status_msg)
            return

def main():
    rclpy.init()
    rclpy.spin(MissionHandler())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

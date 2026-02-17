#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from std_msgs.msg import Bool, String, Float32, Int16
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
import numpy as np
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
        self.last_idx = -1
        self.time_stamp = 0
        self.state = "RUNNING"
        self.pose_actual = [0]*6
        self.checkpoint = 0  # ✅ inicializado
        self.magnitud = 1

        self.alineado = False # Bandera del nodo analyzer que detecta si se alineo con el objeto
        self.servoing_complete = False # Bandera del nodo analyzer que detecta si llego al objetivo
        self.direccion = 0.0

        self.create_subscription(Bool, 'checkpoint', self.checkpoint_callback, 10)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.mission_sub = self.create_subscription(String, 'load_mission', self.load_mission_callback, 10)

        self.wp_pub = self.create_publisher(PoseStamped, 'waypoint', 10)
        self.bezier_pub = self.create_publisher(PoseArray, 'bezier_waypoints', 10)
        self.checkpoint_pub = self.create_publisher(Bool, 'checkpoint', 10)
        self.status_pub = self.create_publisher(Int16, 'mission_status', 10)
        self.picture_pub = self.create_publisher(Int16, 'take_picture', 10)

        self.create_subscription(Bool, 'alineado', self.alineado_callback, 10)
        self.create_subscription(Bool, 'servoing_complete', self.servoing_complete_callback, 10)
        self.create_subscription(Int16, 'direccion', self.direccion_callback, 10)
        self.create_subscription(Int16, 'picture_analized', self.picture_analized_callback, 10)

        self.timer = self.create_timer(0.1, self.run)

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

    def picture_analized_callback(self, msg: Int16):
        self.picture_status = msg.data

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
                # self.run()
        else:
            self.get_logger().error(f"No existe el archivo: {new_path}")

    def alineado_callback(self, msg: Bool):
        self.alineado = msg.data
    
    def servoing_complete_callback(self, msg: Bool):
        self.servoing_complete = msg.data

    def direccion_callback(self, msg: Int16):
        self.direccion = msg.data

        # if self.direccion > 3.0:
        #     self.direccion = 0.0


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
            self.checkpoint = 0
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
            # self.get_logger().info(f"Waypoint {wp[0]}, {wp[1]}, {wp[2]}, {wp[3]}, {wp[4]}, {wp[5]}")

            if self.checkpoint == 1:
                self.checkpoint = 0
                self.idx += 1
        
        elif action["type"] == "bezier_wp":
            wp = action["waypoints"]
            msg = PoseArray()
            if self.last_idx != self.idx:
                self.time_stamp = self.get_clock().now().to_msg()
                self.last_idx = self.idx
            msg.header.stamp = self.time_stamp
            msg.header.frame_id = "world"

            for i in wp:
                p = Pose()

                p.position.x = float(i[0])
                p.position.y = float(i[1])
                p.position.z = float(i[2])
                p.orientation.x = 0.0
                p.orientation.y = 0.0
                p.orientation.z = 0.0
                p.orientation.w = 1.0

                msg.poses.append(p)

            self.bezier_pub.publish(msg)
            
                

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
        
        elif action["type"] == "servo":
            if not hasattr(self, 'servo_state'):
                self.get_logger().info("🔄 Iniciando ciclo de Servo Visual...")
                self.servo_state = "WAITING_PHOTO"
                self.picture_status = 0
                self.picture_pub.publish(Int16(data=1))
                return
            
            if self.servo_state == "WAITING_PHOTO":
                if self.picture_status == 2: # YOLO terminó
                    if self.servoing_complete:
                        self.get_logger().info("🏁 ¡OBJETIVO ALCANZADO!")
                        del self.servo_state
                        self.idx += 1
                        return
                    
                    if not self.alineado:
                        self.servo_state = "ROTATING"
                    else:
                        self.servo_state = "MOVING_FORWARD"
                return
            
            if self.servo_state == "ROTATING":
                correction_rad = float(self.direccion) * (np.pi / 180.0)
                new_yaw = self.pose_actual[5] + correction_rad
                
                self.get_logger().info(f"📐 Alineando: Girando {self.direccion} grados...")
                
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "world"
                # MANTENEMOS la posición actual (X, Y)
                msg.pose.position.x = self.pose_actual[0]
                msg.pose.position.y = self.pose_actual[1]
                msg.pose.position.z = self.pose_actual[2]
                
                q = quaternion_from_euler(self.pose_actual[3], self.pose_actual[4], new_yaw)
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
                
                self.wp_pub.publish(msg)
                self.checkpoint = 0
                self.servo_state = "WAITING_CHECKPOINT"
                return
            
            if self.servo_state == "MOVING_FORWARD":
                dist_paso = 1.5
                yaw_actual = self.pose_actual[5]
                
                self.get_logger().info(f"🚀 Avanzando {dist_paso} metros...")
                
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "world"
                # AVANZAMOS manteniendo el ángulo actual
                msg.pose.position.x = self.pose_actual[0] + dist_paso * np.cos(yaw_actual)
                msg.pose.position.y = self.pose_actual[1] + dist_paso * np.sin(yaw_actual)
                msg.pose.position.z = self.pose_actual[2]
                
                # Mantenemos la orientación actual
                q = quaternion_from_euler(self.pose_actual[3], self.pose_actual[4], yaw_actual)
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
                
                self.wp_pub.publish(msg)
                self.checkpoint = 0
                self.servo_state = "WAITING_CHECKPOINT"
                return
            
            

        
        # elif action["type"] == "servo":
        #     self.get_logger().info("Iniciando servo")

        #     if not hasattr(self, 'waiting_for_analizer'):
        #         self.get_logger().info("📸 Iniciando Servo: Disparando foto y esperando a YOLO...")
        #         self.waiting_for_analizer = True
        #         self.picture_status = 0 # Resetear cualquier estado previo
                
        #         take_picture_msg = Int16(data = 1)
        #         self.picture_pub.publish(take_picture_msg)
        #         return
            
        #     if self.picture_status == 2:
        #         self.get_logger().info("✅ YOLO terminó el análisis. Continuando misión.")
                
        #         if self.servoing_complete:
        #             self.get_logger().info("🏁 ¡Objetivo alcanzado! Fin del servo.")
        #             del self.waiting_for_analizer
        #             self.idx += 1
        #             return
                
        #         dist_paso = 1.5
                
        #         correction_rad = float(self.direccion) * (np.pi / 180.0) # se convierte el angulo de error de deg a rad
        #         new_yaw = self.pose_actual[5] + correction_rad

        #         self.get_logger().info(f"🔄 Aplicando corrección: {self.direccion} deg ({correction_rad:.3f} rad)")
                
        #         msg = PoseStamped()
        #         msg.header.stamp = self.get_clock().now().to_msg()
        #         msg.header.frame_id = "world"

        #         # Mantenemos posición, solo cambiamos orientación
        #         msg.pose.position.x = self.pose_actual[0] + dist_paso * np.cos(new_yaw)
        #         msg.pose.position.y = self.pose_actual[1] + dist_paso * np.sin(new_yaw)
        #         msg.pose.position.z = self.pose_actual[2]

        #         q = quaternion_from_euler(self.pose_actual[3], self.pose_actual[4], new_yaw)
        #         msg.pose.orientation.x = q[0]
        #         msg.pose.orientation.y = q[1]
        #         msg.pose.orientation.z = q[2]
        #         msg.pose.orientation.w = q[3]

        #         # Publicamos para que el submarino gire
        #         self.wp_pub.publish(msg)

        #         self.picture_status = 0

        #         if hasattr(self, 'picture_status') and self.picture_status == 3:
        #             if self.checkpoint == 1:
        #                 self.get_logger().info("🎯 Alineación completada. Continuando misión.")
        #                 self.checkpoint = 0
        #                 del self.waiting_for_analizer
        #                 self.idx += 1

            
            

def main():
    rclpy.init()
    rclpy.spin(MissionHandler())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
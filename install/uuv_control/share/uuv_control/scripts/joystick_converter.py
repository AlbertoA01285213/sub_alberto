#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int16
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control_node')

        self.jsub = self.create_subscription(Joy, '/joy', self.convert_joy, 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.objective_pub = self.create_publisher(Pose, 'pose_objetivo', 10)
        self.picture_pub = self.create_publisher(Int16, 'take_picture', 10)

        # Sensibilidad: cuánto se mueve el setpoint en cada frame
        self.multiplicador_tras = 0.1 
        self.multiplicador_rota = 0.007

        self.pose = Pose()
        self.pose_objetivo = [0.0]*6 # [x, y, z, roll, pitch, yaw]
        self.pose_actual = [0.0]*6

        self.last_picture_time = self.get_clock().now()
        self.cooldown_duration = 0.5

        # --- MAPEO XBOX SERIES X ---
        self.axis_surge = 1     # Stick Izquierdo (Arriba/Abajo)
        self.axis_sway  = 0     # Stick Izquierdo (Izq/Der)
        self.axis_yaw   = 3     # Stick Derecho (Izq/Der)
        # self.axis_heave_p = 4     # Boton (Arriba/Abajo)
        # self.axis_heave_n = 5     # Boton (Arriba/Abajo)
        self.axis_heave = 7
        
        self.btn_picture = 0   # Botón de menú/start
        self.return_home = 1
    
    def pose_callback(self, msg: PoseStamped):
        try:
            self.pose_actual[0] = msg.pose.position.x
            self.pose_actual[1] = msg.pose.position.y
            self.pose_actual[2] = msg.pose.position.z
            quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            self.pose_actual[3] = roll
            self.pose_actual[4] = pitch
            self.pose_actual[5] = yaw

        except Exception as e:
            self.get_logger().error(f"Error en pose_callback: {e}")

    def convert_joy(self, msg):
        # SEGURIDAD: El eje 7 es el D-pad. Si no hay al menos 8 ejes, ignoramos.
        if len(msg.axes) <= 7:
            return
        
        if msg.buttons[self.btn_picture] == 1:
            self.get_logger().info("Foto tomada!")
            current_time = self.get_clock().now()
            # Calculamos la diferencia de tiempo en nanosegundos y pasamos a segundos
            elapsed_time = (current_time - self.last_picture_time).nanoseconds / 1e9
            
            if elapsed_time >= self.cooldown_duration:
                self.picture_pub.publish(Int16(data=1))
                self.get_logger().info("📸 ¡Foto tomada!")
                self.last_picture_time = current_time

        if msg.buttons[self.return_home] == 1:
            self.pose.position.x = 0.0
            self.pose.position.y = 0.0
            self.pose.position.z = 0.0

            self.pose.orientation.x = 0.0
            self.pose.orientation.y = 0.0
            self.pose.orientation.z = 0.0
            self.pose.orientation.w = 1.0

        
        # 1. Calculamos cuánto queremos movernos
        dx = msg.axes[self.axis_surge] * self.multiplicador_tras
        dy = msg.axes[self.axis_sway] * self.multiplicador_tras

        

        # dz = (-1* msg.buttons[self.axis_heave_p] + msg.buttons[self.axis_heave_n])*self.multiplicador_tras

        dz = msg.axes[self.axis_heave] * self.multiplicador_tras
        dyaw = msg.axes[self.axis_yaw] * self.multiplicador_rota

        dx_global = (dx*np.cos(self.pose_actual[5]) - dy*np.sin(self.pose_actual[5])) * self.multiplicador_tras
        dy_global = (dx*np.sin(self.pose_actual[5]) + dy*np.cos(self.pose_actual[5])) * self.multiplicador_tras
        
        # 2. Actualizamos el setpoint acumulativo
        self.pose_objetivo[0] += dx_global
        self.pose_objetivo[1] += dy_global
        self.pose_objetivo[2] += dz
        self.pose_objetivo[5] += dyaw # Solo rotación en Z (Yaw)

        # 3. Llenamos el mensaje Pose
        self.pose.position.x = float(self.pose_objetivo[0])
        self.pose.position.y = float(self.pose_objetivo[1])
        self.pose.position.z = float(self.pose_objetivo[2])

        # Convertimos la rotación Yaw a Quaternion
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.pose_objetivo[5])

        self.pose.orientation.x = qx
        self.pose.orientation.y = qy
        self.pose.orientation.z = qz
        self.pose.orientation.w = qw

        # 4. Publicamos la Pose
        self.objective_pub.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from math import pi, fabs

# Mensajes estándar
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
# Nota: tf_transformations requiere instalar: sudo apt install ros-<distro>-tf-transformations
from tf_transformations import euler_from_quaternion

class ASMC_Axis:
    """Clase auxiliar para manejar la lógica de 1 solo eje (X, o Y, o Z...)"""
    def __init__(self, dt, lmbda, k2, k_alpha, k1_init, k_min, mu, is_angular=False):
        self.dt = dt
        self.lmbda = lmbda
        self.k2 = k2
        self.k_alpha = k_alpha
        self.k_min = k_min
        self.mu = mu
        self.is_angular = is_angular

        self.q_d = 0.0
        self.q_dot_d = 0.0
        self.k1 = k1_init
        self.dot_k1 = 0.0
        self.prev_dot_k1 = 0.0
        self.ua = 0.0

    def update_setpoint(self, q_d, q_dot_d=0.0):
        self.q_d = q_d
        self.q_dot_d = q_dot_d

    def calculate_aux_control(self, q, q_dot):
        self.prev_dot_k1 = self.dot_k1
        
        e1 = self.q_d - q
        e2 = self.q_dot_d - q_dot

        if self.is_angular:
            # Normalización de ángulos a (-pi, pi)
            e1 = (e1 + pi) % (2 * pi) - pi
            e2 = (e2 + pi) % (2 * pi) - pi

        s = e2 + self.lmbda * e1

        # Ley adaptativa para K1
        if self.k1 > self.k_min:
            self.dot_k1 = self.k_alpha * np.sign(fabs(s) - self.mu)
        else:
            self.dot_k1 = self.k_min

        # Integración de la ganancia k1
        self.k1 += (self.dot_k1 + self.prev_dot_k1) / 2.0 * self.dt
        
        # Ley de control ua
        sig_s = np.sign(s) * np.power(fabs(s), 0.5)
        self.ua = -self.k1 * sig_s - self.k2 * s
        return self.ua

class ASMCSubmarineNode(Node):
    def __init__(self):
        super().__init__('asmc_node')

        # 1. Parámetros (Con valores default)
        self.declare_parameter('lambda', [1.0] * 6)
        self.declare_parameter('K1_init', [0.5] * 6)
        self.declare_parameter('K2', [0.1] * 6)
        self.declare_parameter('K_alpha', [0.01] * 6)
        self.declare_parameter('K_min', [0.1] * 6)
        self.declare_parameter('mu', [0.05] * 6)

        dt = 0.1 # 10Hz

        # Inicializar controladores para cada eje
        l = self.get_parameter('lambda').value
        k1 = self.get_parameter('K1_init').value
        k2 = self.get_parameter('K2').value
        ka = self.get_parameter('K_alpha').value
        km = self.get_parameter('K_min').value
        mu = self.get_parameter('mu').value

        self.controllers = [
            ASMC_Axis(dt, l[i], k2[i], ka[i], k1[i], km[i], mu[i], is_angular=(i>=3))
            for i in range(6)
        ]

        # 2. Variables de estado
        self.pose_actual = np.zeros(6)
        self.pose_objetivo = np.zeros(6)
        self.f = np.zeros(6)
        self.g = np.eye(6)
        self.q_dot_dot_d = np.zeros(6) # Aceleración deseada (usualmente 0)
        self.max_tau = [127.0, 34.0, 118.0, 28.0, 9.6, 36.6]

        # 3. Suscriptores y Publicadores
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.objetivo_sub = self.create_subscription(Pose, 'pose_objetivo', self.pose_objetivo_callback, 10)

        self.vel_sub = self.create_subscription(Odometry, '/odom', self.velocity_callback, 10)
        self.velocidad_actual = np.zeros(6)

        # Recibimos un MultiArray para dynamics
        self.dynamics_sub = self.create_subscription(Float64MultiArray, 'dynamics', self.dynamics_callback, 10)
        
        self.thruster_pub = self.create_publisher(Float64MultiArray, '/forces', 10)

        # 4. Timer de control
        self.create_timer(dt, self.control_loop)

    def pose_callback(self, msg: Pose):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r, p, y = euler_from_quaternion(quat)
        self.pose_actual = np.array([msg.position.x, msg.position.y, msg.position.z, r, p, y])

    def pose_objetivo_callback(self, msg: Pose):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r, p, y = euler_from_quaternion(quat)
        self.pose_objetivo = np.array([msg.position.x, msg.position.y, msg.position.z, r, p, y])
        
        for i in range(6):
            self.controllers[i].update_setpoint(self.pose_objetivo[i])

    def velocity_callback(self, msg: Twist):
        # 1. Extraer Pose (Posición y Orientación)
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        quat = [o.x, o.y, o.z, o.w]
        r, p_angle, y = euler_from_quaternion(quat)
        
        self.pose_actual = np.array([p.x, p.y, p.z, r, p_angle, y])

        # 2. Extraer Velocidad (Linear y Angular)
        # IMPORTANTE: Estas velocidades ya están en el "Body Frame"
        v = msg.twist.twist.linear
        a = msg.twist.twist.angular
        
        self.velocidad_actual = np.array([v.x, v.y, v.z, a.x, a.y, a.z])
        
    def dynamics_callback(self, msg: Float64MultiArray):
        # Asumiendo que los primeros 36 son la matriz G (6x6) y los siguientes 6 son el vector F
        if len(msg.data) >= 42:
            self.g = np.array(msg.data[0:36]).reshape((6, 6))
            self.f = np.array(msg.data[36:42])

    def control_loop(self):
        phi = self.pose_actual[3]   # Roll
        theta = self.pose_actual[4] # Pitch
        psi = self.pose_actual[5]   # Yaw

        # --- 2. CÁLCULO DEL ERROR GLOBAL ---
        # Diferencia entre donde quiero estar y donde estoy (World Frame)
        error_pos_global = self.pose_objetivo[0:3] - self.pose_actual[0:3]
        error_ori_global = self.pose_objetivo[3:6] - self.pose_actual[3:6]

        # Normalización del error de Yaw (MUY IMPORTANTE para evitar la espiral)
        error_ori_global[2] = (error_ori_global[2] + pi) % (2 * pi) - pi

        # --- 3. TRANSFORMACIÓN AL MARCO DEL CUERPO (Body Frame) ---
        # Matriz de rotación simplificada (solo Yaw es la más crítica para la espiral)
        # Si quieres precisión total, se usa la matriz R completa que tienes en el C++
        R_yaw = np.array([
            [ np.cos(psi), np.sin(psi), 0],
            [-np.sin(psi), np.cos(psi), 0],
            [ 0,           0,           1]
        ])
        
        # Proyectamos el error de posición al frente y lado del submarino
        error_pos_body = R_yaw @ error_pos_global

        # 1. Calcular ua para cada eje (asumiendo velocidad 0 si no tienes sensor de velocidad)
        ua_vec = np.zeros(6)
        for i in range(3):
            self.controllers[i].update_setpoint(error_pos_body[i], 0.0)
            ua_vec[i] = self.controllers[i].calculate_aux_control(0.0, self.velocidad_actual[i])
        
        for i in range(3, 6):
            self.controllers[i].update_setpoint(error_ori_global[i-3], 0.0)
            ua_vec[i] = self.controllers[i].calculate_aux_control(0.0, self.velocidad_actual[i])
        
        self.get_logger().info("Control loop")
        

        # 2. Ley de Control de Manipulación
        try:
            # self.get_logger().info(f"G: {len(self.g)}")
            self.get_logger().info(f"G: {abs(np.linalg.det(self.g))}")
            # Evitar división por cero o matrices singulares
            # if abs(np.linalg.det(self.g)) > 1e-5:
            #     self.get_logger().info("Ley de control")

            #     # u = G^-1 * (q_dd - f - ua)
            #     u = np.linalg.inv(self.g) @ (self.q_dot_dot_d - self.f - ua_vec)

            #     # 3. Saturación
            #     for i in range(6):
            #         u[i] = np.clip(u[i], -self.max_tau[i], self.max_tau[i])
                
            #     # 4. Publicar
            #     thrust_msg = Float64MultiArray()
            #     thrust_msg.data = u.tolist()
            #     self.thruster_pub.publish(thrust_msg)

            self.get_logger().info("Ley de control")

            # u = G^-1 * (q_dd - f - ua)
            u = np.linalg.inv(self.g) @ (self.q_dot_dot_d - self.f - ua_vec)

            # 3. Saturación
            for i in range(6):
                u[i] = np.clip(u[i], -self.max_tau[i], self.max_tau[i])
            
            # 4. Publicar
            thrust_msg = Float64MultiArray()
            thrust_msg.data = u.tolist()
            self.thruster_pub.publish(thrust_msg)
                
        except np.linalg.LinAlgError:
            self.get_logger().error("Matriz G es singular, no se puede calcular el control.")

def main(args=None):
    rclpy.init(args=args)
    node = ASMCSubmarineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
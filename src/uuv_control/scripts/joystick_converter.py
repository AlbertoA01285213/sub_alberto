#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Int16, Float64MultiArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
# import math

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control_node')

        self.jsub = self.create_subscription(Joy, '/joy', self.convert_joy, 10)

        self.objective_pub = self.create_publisher(Pose, 'pose_objetivo', 10)
        self.picture_pub = self.create_publisher(Int16, 'take_picture', 10)

        self.multiplicador_tras = 100
        self.multiplicador_rota = 2

        self.pose = Pose()
        self.pose_objetivo = [0.0]*6

        self.joy = Float64MultiArray()
        self.joy.data = [0.0]*6

        self.surge = 1     # Adelante y atras
        self.sway = 0      # Izquierda y derecha
        self.heave_p = 10  # Arriba
        self.heave_n = 9   # Abajo
        self.yaw = 2       # Girar izquieda y derecha

        self.picture = 7    # Tomar foto
    
    def convert_joy(self, msg):
        if len(msg.axes) < 6:
            return
        
        self.joy.data[0] = float(msg.axes[self.surge] * self.multiplicador_tras)                         # Adelante y atras
        self.joy.data[1] = float(msg.axes[self.sway] * self.multiplicador_tras)                          # Izquiera y derecha
        self.joy.data[2] = float(((msg.axes[self.yaw] - msg.axes[5])/2) * self.multiplicador_tras)       # Arriba y abajo
        # self.joy.data[3] = float((-msg.buttons[4] + msg.buttons[5]) * self.multiplicador_rota)      # Desactivado ya que no se necesita
        # self.joy.data[4]= float(msg.axes[heave_p] * self.multiplicador_rota)                        # Desactivado ya que no se necesita
        self.joy.data[5] = float(msg.axes[self.pitch] * self.multiplicador_rota)                         # Rotar

        self.pose_objetivo[0] = self.joy.data[0] + self.pose_objetivo[0]
        self.pose_objetivo[1] = self.joy.data[1] + self.pose_objetivo[1]
        self.pose_objetivo[2] = self.joy.data[2] + self.pose_objetivo[2]
        self.pose_objetivo[3] = self.joy.data[3] + self.pose_objetivo[3]
        self.pose_objetivo[4] = self.joy.data[4] + self.pose_objetivo[4]
        self.pose_objetivo[5] = self.joy.data[5] + self.pose_objetivo[5]

        self.pose.position.x = self.pose_objetivo[0]
        self.pose.position.y = self.pose_objetivo[1]
        self.pose.position.z = self.pose_objetivo[2]

        rx, ry, rz, rw = quaternion_from_euler(self.pose_objetivo[3], self.pose_objetivo[4], self.pose_objetivo[5])

        self.pose.orientation.x = rx
        self.pose.orientation.y = ry
        self.pose.orientation.z = rz
        self.pose.orientation.w = rw

        self.objective_pub.publish(self.pose_objetivo)

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
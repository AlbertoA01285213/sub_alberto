#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64MultiArray

import numpy as np
# import math

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control_node')
        self.jsub = self.create_subscription(Joy, '/joy', self.convert_joy, 10)

        self.publisher = self.create_publisher(Float64MultiArray, 'uuv/forces', 10)
        self.vel_msg = Pose()

        self.objective_pub = self.create_publisher(Pose, 'pose_objetivo', 10)

        self.multiplicador_tras = 100
        self.multiplicador_rota = 2

        self.pose_objetivo = [0.0]*6

        self.joy = Float64MultiArray()
        self.joy.data = [0.0]*6

        self.surge = 1     # Adelante y atras
        self.sway = 0      # Izquierda y derecha
        self.heave_p = 10  # Arriba
        self.heave_n = 9   # Abajo

        self.yaw = 2       # Girar izquieda y derecha

        self.picture =     # Tomar foto
    
    def convert_joy(self, msg):
        surge = 1
        sway = 0
        heave_p = 4
        heave_n = 5

        pitch = 3
        roll_p = 10
        roll_n = 9
        yaw = 2
        
        self.joy.data[0] = float(msg.axes[surge] * self.multiplicador_tras) # listo
        self.joy.data[1] = float(msg.axes[sway] * self.multiplicador_tras) # listo
        self.joy.data[2] = float(((msg.axes[yaw] - msg.axes[5])/2) * self.multiplicador_tras)
        self.joy.data[3] = float((-msg.buttons[4] + msg.buttons[5]) * self.multiplicador_rota)# - msg.buttons[roll_n])
        self.joy.data[4]= float(msg.axes[heave_p] * self.multiplicador_rota)# ;isto
        self.joy.data[5] = float(msg.axes[pitch] * self.multiplicador_rota) # listo

        self.pose_objetivo[0] = self.joy.data[0] + self.pose_objetivo[0]
        self.pose_objetivo[1] = self.joy.data[1] + self.pose_objetivo[1]
        self.pose_objetivo[2] = self.joy.data[2] + self.pose_objetivo[2]
        self.pose_objetivo[3] = self.joy.data[3] + self.pose_objetivo[3]

        self.objective_pub.publish(self.pose_objetivo)






        self.publisher.publish(self.joy)

def main(args=None):
    rclpy.init(args=args)
    rmn = TeleopControl()
    rclpy.spin(rmn)
    rmn.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
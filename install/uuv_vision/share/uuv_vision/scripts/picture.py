#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import cv2
import message_filters
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, String, Float32, Int16
from sensor_msgs.msg import Image
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
from ament_index_python.packages import get_package_share_directory

class Picture(Node):
    def __init__(self):
        super().__init__('Picture_node')
        self.bridge = CvBridge()
        self.count = 0

        self.left_sub = message_filters.Subscriber(self, Image, '/camera/left')
        self.right_sub = message_filters.Subscriber(self, Image, '/camera/right')
        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

        self.create_subscription(Int16, 'take_picture', self.trigger_callback, 10)
        self.img_left_pub = self.create_publisher(Image, 'image_left_to_analyze', 10)
        self.img_right_pub = self.create_publisher(Image, 'image_right_to_analyze', 10)
        
        self.latest_left = None
        self.latest_right = None

        self.timer = self.create_timer(1.0, self.run)

    def sync_callback(self, left_msg, right_msg):
        self.latest_left = left_msg
        self.latest_right = right_msg

    def trigger_callback(self, msg):
        tomar_foto = msg.data
        if tomar_foto == 1 and self.latest_left is not None:
            self.img_left_pub.publish(self.latest_left)
            self.img_right_pub.publish(self.latest_right)
            self.get_logger().info("Foto enviada al analizador")

            try:
                cv_img_l = self.bridge.imgmsg_to_cv2(self.latest_left, desired_encoding='bgr8')
                cv_img_r = self.bridge.imgmsg_to_cv2(self.latest_right, desired_encoding='bgr8')
                
                conc_img = cv2.hconcat([cv_img_l, cv_img_r])
                
                # CORRECCIÓN: Ruta completa con nombre de archivo y extensión .jpg
                filename = f"captura_uuv_{self.count}.jpg"
                # save_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'Fotos_sub', filename)
                
                save_path = os.path.join(get_package_share_directory('uuv_vision'), 'images', filename)
                cv2.imwrite(save_path, conc_img)
                self.get_logger().info(f"Imagen guardada en: {save_path}")
                
                self.count += 1
                tomar_foto = 0
            except Exception as e:
                self.get_logger().error(f"Error al guardar imagen: {e}")


    def run(self):
        cv_img_l = self.bridge.imgmsg_to_cv2(self.latest_left, desired_encoding='bgr8')
        filename = f"captura_uuv_{self.count}.jpg"
        save_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'Fotos_sub', filename)
        
        # cv2.imwrite(save_path, cv_img_l)
        # self.get_logger().info(f"Imagen guardada en: {save_path}")
        
        self.count += 1


def main():
    rclpy.init()
    rclpy.spin(Picture())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import TransformBroadcaster

class PoseToTfPublisher(Node):

    def __init__(self):
        super().__init__('pose_to_tf_publisher')

        # --- Parámetros ---
        self.declare_parameter('pose_topic', "/pose")
        self.declare_parameter('parent_frame', "world")
        self.declare_parameter('child_frame', "base_link")

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        # --- Broadcaster TF ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Suscripción ---
        self.pose_subscriber = self.create_subscription(
            Pose,
            pose_topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(f"Nodo iniciado. Escuchando {pose_topic}")

    def pose_callback(self, msg: Pose):

        # Crear transformada vacía
        t = TransformStamped()

        # Pose NO tiene header, así que generamos timestamp y frame
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Posición
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z

        # Orientación
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        # Publicar transformada
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()

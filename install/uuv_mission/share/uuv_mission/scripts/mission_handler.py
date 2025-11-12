#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time

class MissionHandler(Node):
    def __init__(self):
        super().__init__('mission_handler')

        self.declare_parameter('mission_file', '/home/alberto/Documents/sub_alberto/src/uuv_mission/missions/mission_1.yaml')
        mission_path = self.get_parameter('mission_file').value

        with open(mission_path, 'r') as f:
            self.mission = yaml.safe_load(f)

        self.actions = self.mission["actions"]
        self.idx = 0
        self.state = "RUNNING"
        self.pose_actual = [0]*6
        self.checkpoint = 0  # ✅ inicializado

        self.create_subscription(Bool, 'checkpoint', self.checkpoint_callback, 10)
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.wp_pub = self.create_publisher(Pose, 'waypoint', 10)

        self.timer = self.create_timer(0.5, self.run)

    def checkpoint_callback(self, msg: Bool):
        self.checkpoint = msg.data  # ✅ corregido

    def pose_callback(self, msg: Pose):
        self.pose_actual[0] = msg.position.x
        self.pose_actual[1] = msg.position.y
        self.pose_actual[2] = msg.position.z
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.pose_actual[3] = roll
        self.pose_actual[4] = pitch
        self.pose_actual[5] = yaw

    def run(self):
        if self.idx >= len(self.actions):
            self.get_logger().info("Mission complete!")
            return

        action = self.actions[self.idx]

        if action["type"] == "goto":
            wp = action["waypoint"]
            msg = Pose()
            msg.position.x = wp[0]
            msg.position.y = wp[1]
            msg.position.z = wp[2]
            msg.orientation.w = 1.0

            self.wp_pub.publish(msg)
            self.get_logger().info(f"Enviando waypoint {wp}")

            if self.checkpoint == 1:
                self.checkpoint = 0
                self.idx += 1

        elif action["type"] == "hold":
            duration = float(action["duration"])
            if not hasattr(self, "hold_start"):
                self.get_logger().info(f"Holding for {duration} seconds")
                self.hold_start = time.perf_counter()

            if time.perf_counter() - self.hold_start >= duration:
                del self.hold_start
                self.idx += 1

        elif action["type"] == "rotate":
            rotations = action["rotation"]

            qx, qy, qz, qw = quaternion_from_euler(
                self.pose_actual[3] + rotations[0],
                self.pose_actual[4] + rotations[1],
                self.pose_actual[5] + rotations[2]
            )

            msg = Pose()
            msg.position.x = self.pose_actual[0]
            msg.position.y = self.pose_actual[1]
            msg.position.z = self.pose_actual[2]
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            self.wp_pub.publish(msg)

            if self.checkpoint == 1:
                self.checkpoint = 0
                self.idx += 1

def main():
    rclpy.init()
    rclpy.spin(MissionHandler())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

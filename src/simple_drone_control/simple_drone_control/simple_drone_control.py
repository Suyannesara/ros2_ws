#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode
import threading
from math import sin, cos


class SimpleDroneControl(Node):
    def __init__(self):
        super().__init__('simple_drone_control')

        self.pub_setpoint = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        self.position = [0.0, 0.0, -4.0]  # Altura inicial (4 metros acima do solo)
        self.yaw = 0.0  # Radianos
        self.rate_hz = 10

        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_messages)

        # Controla o loop de input num thread separado
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    def publish_messages(self):
        # Publica OffboardControlMode com position=True para controlar posição/yaw
        mode_msg = OffboardControlMode()
        mode_msg.position = True
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        self.pub_mode.publish(mode_msg)

        # Publica setpoint atual
        sp = TrajectorySetpoint()
        sp.position = self.position
        sp.yaw = self.yaw
        self.pub_setpoint.publish(sp)

    def keyboard_loop(self):
        print("Comandos: dir (90° direita), esq (90° esquerda), front (1m frente), back (1m trás), exit (sair)")
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "dir":
                self.yaw -= 0.785398  # -45 graus em rad
                print(f"Virando para a direita. Yaw atual: {self.yaw:.2f} rad")
            elif cmd == "esq":
                self.yaw += 0.785398  # +45 graus em rad
                print(f"Virando para a esquerda. Yaw atual: {self.yaw:.2f} rad")
            elif cmd == "front":
                # mover 1m para frente, na direção do yaw atual
                dx = 5.0 * cos(self.yaw)
                dy = 5.0 * sin(self.yaw)
                self.position[0] += dx
                self.position[1] += dy
                print(f"Indo 1m para frente para {self.position[0]:.2f}, {self.position[1]:.2f}")
            elif cmd == "back":
                dx = -5.0 * cos(self.yaw)
                dy = -5.0 * sin(self.yaw)
                self.position[0] += dx
                self.position[1] += dy
                print(f"Indo 1m para trás para {self.position[0]:.2f}, {self.position[1]:.2f}")
            elif cmd == "exit":
                print("Saindo...")
                rclpy.shutdown()
                break
            else:
                print("Comando inválido.")

def main(args=None):
    from math import sin, cos

    rclpy.init(args=args)
    node = SimpleDroneControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

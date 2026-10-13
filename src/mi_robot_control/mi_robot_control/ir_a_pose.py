#!/usr/bin/env python3
"""
ir_a_pose.py — Llevar el robot diferencial a (x, y, θ).

    ros2 run mi_robot_control ir_a_pose --ros-args -p x:=2.0 -p y:=0.5 -p theta:=90.0

Fase 1: ir a (x, y) con Line of Sight, ec. (17) y (21).
Fase 2: ya en el punto, girar en el sitio hasta la orientación θ deseada:
        r = λ3 (θd − φ)
(Con LOS solo no se puede imponer la orientación final: el robot no se mueve de lado.)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def envolver(a):
    """Ángulo en [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


class IrAPose(Node):
    def __init__(self):
        super().__init__('ir_a_pose')

        # Meta
        self.xd = self.declare_parameter('x', 2.0).value
        self.yd = self.declare_parameter('y', 0.0).value
        self.thd = math.radians(self.declare_parameter('theta', 0.0).value)  # en grados

        # Ganancias Λ = diag(λ1, λ2, λ3)
        self.l1, self.l2, self.l3 = 0.5, 0.5, 0.5
        self.u_max, self.r_max = 0.3, 1.5     # saturación [m/s], [rad/s]
        self.tol_pos = 0.03                   # [m]
        self.tol_ang = math.radians(2.0)      # [rad]

        self.fase = 1
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.02, self.lazo)    # 50 Hz

        self.get_logger().info(
            f'Meta: x={self.xd} m, y={self.yd} m, θ={math.degrees(self.thd):.0f}°')

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        phi = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.pose = (p.x, p.y, phi)

    def lazo(self):
        if self.pose is None:
            return
        x, y, phi = self.pose
        u, r = 0.0, 0.0

        # ---------- Fase 1: llegar a (x, y) con Line of Sight ----------
        if self.fase == 1:
            ex, ey = self.xd - x, self.yd - y                 # error de posición
            if math.hypot(ex, ey) < self.tol_pos:
                self.fase = 2
                self.get_logger().info('Llegó a (x, y). Girando a θ...')
            else:
                phi_d = math.atan2(ey, ex)                     # ec. (17)
                e_phi = envolver(phi_d - phi)
                # ec. (21) con η̇d = 0 (set-point):
                u = math.cos(phi) * self.l1 * ex + math.sin(phi) * self.l2 * ey
                r = self.l3 * e_phi
                u = max(u, 0.0)                                # si está atrás, primero gira

        # ---------- Fase 2: girar en el sitio hasta θd ----------
        if self.fase == 2:
            e_th = envolver(self.thd - phi)
            if abs(e_th) < self.tol_ang:
                self.fase = 3
                self.get_logger().info(
                    f'Meta alcanzada: x={x:.3f}, y={y:.3f}, θ={math.degrees(phi):.1f}°')
            else:
                r = self.l3 * e_th

        # Saturación y envío
        cmd = Twist()
        cmd.linear.x = max(-self.u_max, min(self.u_max, u))
        cmd.angular.z = max(-self.r_max, min(self.r_max, r))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    nodo = IrAPose()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub.publish(Twist())   # detener el robot
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

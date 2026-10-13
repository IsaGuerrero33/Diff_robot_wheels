#!/usr/bin/env python3
"""
control_los.py — Control cinemático (Line of Sight) para mi_robot_diferencial

Implementa la ley de la clase, ec. (21):

    [u]   [cos φ  sin φ  0] ( [ẋd]   [λ1  0   0 ] [xd - x] )
    [r] = [  0      0    1] ( [ẏd] + [ 0  λ2  0 ] [yd - y] )
                            ( [φ̇d]   [ 0   0  λ3] [φd - φ] )
          \____ J(φ)⁺ _____/

con φd por Line of Sight (ec. 17) y luego ω = W⁺ ξ con la W del diferencial (ec. 22).

Lazo cerrado:
    Gazebo (DiffDrive) --/odom--> control_los --/cmd_vel--> DiffDrive --> ruedas

Modos (parámetro 'modo'):
    setpoint   : ir a un punto. La meta se da con "2D Goal Pose" en RViz (/goal_pose).
                 η̇d = 0  ->  control P puro (ec. 12).
    slalom     : recorrer el pasillo de la pista haciendo zig-zag (feedforward + P, ec. 13).
    lemniscata : seguir un "8" (feedforward + P, ec. 13).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray

# ===== Parámetros físicos (deben coincidir con el xacro) =====
A_RUEDA = 0.05          # a : rueda_radio
D_MEDIA = 0.25 / 2.0    # d : separacion_ruedas / 2

# ===== Ec. (22): ξ = [u, v, r]ᵀ = W ω,  con ω = [ω_izq, ω_der]ᵀ =====
W = np.array([[ A_RUEDA / 2.0,             A_RUEDA / 2.0],
              [ 0.0,                       0.0],
              [-A_RUEDA / (2.0 * D_MEDIA), A_RUEDA / (2.0 * D_MEDIA)]])
W_PINV = np.linalg.pinv(W)   # ω = W⁺ ξ


def envolver(a):
    """Envuelve un ángulo a [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ley_control(pose, ref, lambdas, u_max, r_max):
    """
    Ec. (21). Función pura (sin ROS) para poder probarla fuera de Gazebo.

    pose = (x, y, φ)           pose medida
    ref  = (xd, yd, ẋd, ẏd)    referencia y su derivada
    Devuelve (u, r, e_x, e_y, e_φ)
    """
    x, y, phi = pose
    xd, yd, xd_dot, yd_dot = ref
    l1, l2, l3 = lambdas

    # Error en el marco mundo: e_η = η_d − η
    ex, ey = xd - x, yd - y

    # Velocidad deseada en el mundo: η̇ = η̇d + Λ e   (ec. 11)
    vx = xd_dot + l1 * ex
    vy = yd_dot + l2 * ey

    # Line of Sight (ec. 17). atan2, no arctan, para no perder el cuadrante.
    # En set-point (η̇d = 0, λ1 = λ2) es exactamente atan2(yd − y, xd − x).
    phi_d = math.atan2(vy, vx)
    e_phi = envolver(phi_d - phi)

    # φ̇d se omite (feedforward angular): el término λ3·e_φ lo compensa.
    phi_d_dot = 0.0

    # Ec. (21): ξ = J(φ)⁺ [η̇d + Λ e]
    J_pinv = np.array([[math.cos(phi), math.sin(phi), 0.0],
                       [0.0,           0.0,           1.0]])
    u, r = J_pinv @ np.array([vx, vy, phi_d_dot + l3 * e_phi])

    # Restricciones prácticas
    u = max(u, 0.0)                         # si la meta está atrás, primero gira
    u = float(np.clip(u, -u_max, u_max))    # saturación (evita el arranque brusco)
    r = float(np.clip(r, -r_max, r_max))
    return u, r, ex, ey, e_phi


class ControlLOS(Node):
    def __init__(self):
        super().__init__('control_los')

        self.declare_parameter('modo', 'setpoint')          # setpoint | slalom | lemniscata
        self.declare_parameter('lambdas', [0.8, 0.8, 2.5])  # diagonal de Λ (ec. 5)
        self.declare_parameter('u_max', 0.30)               # [m/s]
        self.declare_parameter('r_max', 2.00)               # [rad/s]
        self.declare_parameter('tolerancia', 0.03)          # radio de llegada [m]
        self.declare_parameter('frecuencia', 50.0)          # [Hz]
        # Slalom por el pasillo de pista.sdf (x de 0 a 4 m, muros en y = ±1 m)
        self.declare_parameter('slalom_largo', 3.5)         # [m]
        self.declare_parameter('slalom_amplitud', 0.4)      # [m]
        self.declare_parameter('slalom_vel', 0.12)          # avance en x [m/s]
        self.declare_parameter('slalom_ondas', 2.0)         # número de zig-zags
        # Lemniscata
        self.declare_parameter('lem_tamano', 0.8)           # [m]
        self.declare_parameter('lem_periodo', 40.0)         # [s]

        gp = lambda n: self.get_parameter(n).value
        self.modo = gp('modo')
        self.lambdas = [float(v) for v in gp('lambdas')]
        self.u_max, self.r_max = gp('u_max'), gp('r_max')
        self.tol = gp('tolerancia')
        self.p = {n: gp(n) for n in ('slalom_largo', 'slalom_amplitud', 'slalom_vel',
                                     'slalom_ondas', 'lem_tamano', 'lem_periodo')}

        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.cb_meta, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_err = self.create_publisher(Vector3, '/error_control', 10)
        self.pub_w = self.create_publisher(Float64MultiArray, '/omega_ruedas', 10)
        self.pub_ref = self.create_publisher(Path, '/trayectoria_ref', 1)
        self.pub_real = self.create_publisher(Path, '/trayectoria_real', 1)

        self.pose = None
        self.meta = None
        self.llego = False
        self.t0 = None
        self.camino_real = Path()
        self.camino_real.header.frame_id = 'odom'
        self.n = 0

        self.create_timer(1.0 / gp('frecuencia'), self.lazo)
        self.get_logger().info(
            f'control_los | modo={self.modo} | Λ=diag{tuple(self.lambdas)} | '
            f'u_max={self.u_max} m/s, r_max={self.r_max} rad/s\nW (ec. 22) =\n{W}')

    # ------------------------------------------------------------------
    def cb_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_de_quat(p.orientation))

    def cb_meta(self, msg):
        self.meta = (msg.pose.position.x, msg.pose.position.y)
        self.llego = False
        self.get_logger().info(f'Nueva meta: ({self.meta[0]:.2f}, {self.meta[1]:.2f})')

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Referencias η_d(t), η̇_d(t)
    # ------------------------------------------------------------------
    def referencia(self, t):
        p = self.p
        if self.modo == 'slalom':
            L, A, v = p['slalom_largo'], p['slalom_amplitud'], p['slalom_vel']
            k = 2.0 * math.pi * p['slalom_ondas'] / L     # número de onda
            t = min(t, L / v)                             # al final se queda quieta
            xd, xd_dot = v * t, (v if t < L / v else 0.0)
            yd = A * math.sin(k * xd)
            yd_dot = A * k * math.cos(k * xd) * xd_dot
        else:  # lemniscata de Gerono, pasa por el origen
            A, w = p['lem_tamano'], 2.0 * math.pi / p['lem_periodo']
            xd, yd = A * math.sin(w * t), 0.5 * A * math.sin(2.0 * w * t)
            xd_dot, yd_dot = A * w * math.cos(w * t), A * w * math.cos(2.0 * w * t)
        return xd, yd, xd_dot, yd_dot

    def publicar_ref_completa(self):
        """Dibuja la trayectoria deseada completa en RViz."""
        camino = Path()
        camino.header.frame_id = 'odom'
        T = (self.p['slalom_largo'] / self.p['slalom_vel']
             if self.modo == 'slalom' else self.p['lem_periodo'])
        for t in np.linspace(0.0, T, 300):
            xd, yd, _, _ = self.referencia(t)
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x, ps.pose.position.y = xd, yd
            camino.poses.append(ps)
        self.pub_ref.publish(camino)

    # ------------------------------------------------------------------
    def lazo(self):
        if self.pose is None:
            return

        # 1) Referencia
        if self.modo == 'setpoint':
            if self.meta is None:
                return
            ref = (self.meta[0], self.meta[1], 0.0, 0.0)     # η̇d = 0
            if math.hypot(ref[0] - self.pose[0], ref[1] - self.pose[1]) < self.tol:
                self.enviar(0.0, 0.0)
                if not self.llego:
                    self.get_logger().info('Meta alcanzada')
                    self.llego = True
                return
        else:
            if self.t0 is None:              # el tiempo arranca con la primera odometría
                self.t0 = self.ahora()
            ref = self.referencia(self.ahora() - self.t0)

        # 2) Ley de control, ec. (21)
        u, r, ex, ey, ephi = ley_control(self.pose, ref, self.lambdas, self.u_max, self.r_max)
        self.enviar(u, r)

        # 3) Para graficar
        self.pub_err.publish(Vector3(x=ex, y=ey, z=ephi))
        omega = W_PINV @ np.array([u, 0.0, r])   # ω = W⁺ ξ  -> lo que hace DiffDrive por dentro
        self.pub_w.publish(Float64MultiArray(data=[float(omega[0]), float(omega[1])]))

        self.n += 1
        if self.n % 10 == 0:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x, ps.pose.position.y = self.pose[0], self.pose[1]
            self.camino_real.poses.append(ps)
            self.pub_real.publish(self.camino_real)
            if self.modo != 'setpoint':
                self.publicar_ref_completa()

    def enviar(self, u, r):
        cmd = Twist()
        cmd.linear.x = u
        cmd.angular.z = r
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    nodo = ControlLOS()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.enviar(0.0, 0.0)
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math

class CinematicaLlantas(Node):
    def __init__(self):
        super().__init__('cinematica_llantas')
        
        # 1. Parámetros físicos del robot (del Xacro)
        self.r = 0.05   # Radio de la llanta
        self.L = 0.23   # Separación entre llantas
        
        # 2. Pose calculada internamente (q = [x, y, theta])
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 3. Pose real entregada por Gazebo (DiffDrive Odom)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        
        # Control de tiempo para la integración (dt)
        self.last_time = None # Cambiamos esto para el seguro inicial
        self.first_time = True # Seguro activado
        
        # 4. Suscriptores (leemos w y leemos la odometría real)
        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # 5. Timer para imprimir el reporte de error cada 1 segundo
        self.timer = self.create_timer(1.0, self.reportar_error)
        
    def joint_cb(self, msg):
        """Lee W de las llantas, calcula U y luego integra Q"""
        w_I = 0.0
        w_D = 0.0
        
        # Extraer velocidades angulares asegurando qué rueda es cuál
        for i, name in enumerate(msg.name):
            if name == 'joint_rueda_izq':
                w_I = msg.velocity[i]
            elif name == 'joint_rueda_der':
                w_D = msg.velocity[i]
                
        # Calcular delta de tiempo (dt)
        current_time = self.get_clock().now()

        # SEGURO DE TIEMPO INICIAL: Evita el salto gigante de 'dt'
        if self.first_time:
            self.last_time = current_time
            self.first_time = False
            return
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # TRANSFORMACIÓN 1: w -> u
        v = (self.r / 2) * w_D + (self.r / 2) * w_I
        omega = (self.r / self.L) * w_D - (self.r / self.L) * w_I
        
        # TRANSFORMACIÓN 2 E INTEGRACIÓN: u -> q_dot -> q
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # LÍMITE DEL CÍRCULO: Mantiene el ángulo siempre entre -pi y pi (igual que Gazebo)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def odom_cb(self, msg):
        """Lee la posición reportada por el plugin de Gazebo para comparar"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        # Convertir Cuaternión a Ángulo Euler (Theta/Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.odom_theta = math.atan2(siny_cosp, cosy_cosp)

    def reportar_error(self):
        """Compara la Q calculada con la Odom de Gazebo y reporta el error"""
        err_x = abs(self.odom_x - self.x)
        err_y = abs(self.odom_y - self.y)
        err_theta = abs(self.odom_theta - self.theta)
        
        self.get_logger().info("--- REPORTE DE CINEMÁTICA ---")
        self.get_logger().info(f"Calc   -> x: {self.x:.3f}, y: {self.y:.3f}, th: {self.theta:.3f}")
        self.get_logger().info(f"Gazebo -> x: {self.odom_x:.3f}, y: {self.odom_y:.3f}, th: {self.odom_theta:.3f}")
        self.get_logger().info(f"ERROR  -> x: {err_x:.4f}, y: {err_y:.4f}, th: {err_theta:.4f}\n")

def main(args=None):
    rclpy.init(args=args)
    nodo = CinematicaLlantas()
    rclpy.spin(nodo)    
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
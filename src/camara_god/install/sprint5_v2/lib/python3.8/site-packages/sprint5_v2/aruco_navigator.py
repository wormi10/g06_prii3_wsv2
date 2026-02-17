#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time
import random
import math
from typing import Optional, Dict, Tuple


class ArucoNavigator(Node):
    """
    Nodo que navega el robot hacia marcadores ArUco de forma aleatoria.
    Selecciona aleatoriamente entre los ArUcos 20, 21, 22, 23 y navega hacia ellos.
    """
    
    def __init__(self):
        super().__init__('aruco_navigator')
        
        # Parámetros de configuración
        self.declare_parameter('camera_resolution_x', 1280)  # Ancho de imagen en píxeles
        self.declare_parameter('camera_resolution_y', 720)   # Alto de imagen en píxeles
        self.declare_parameter('real_world_width', 2.0)      # Ancho real del área en metros
        self.declare_parameter('real_world_height', 1.5)     # Alto real del área en metros
        self.declare_parameter('goal_tolerance', 0.1)        # Tolerancia para considerar objetivo alcanzado (metros)
        self.declare_parameter('linear_speed_max', 0.5)      # Velocidad lineal máxima (m/s)
        self.declare_parameter('angular_speed_max', 1.0)     # Velocidad angular máxima (rad/s)
        
        # Obtener parámetros
        self.cam_res_x = self.get_parameter('camera_resolution_x').value
        self.cam_res_y = self.get_parameter('camera_resolution_y').value
        self.real_width = self.get_parameter('real_world_width').value
        self.real_height = self.get_parameter('real_world_height').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear_speed = self.get_parameter('linear_speed_max').value
        self.max_angular_speed = self.get_parameter('angular_speed_max').value
        
        # Factor de conversión píxeles -> metros
        self.px_to_m_x = self.real_width / self.cam_res_x
        self.px_to_m_y = self.real_height / self.cam_res_y
        
        self.get_logger().info(f'Conversión px->m: X={self.px_to_m_x:.4f} Y={self.px_to_m_y:.4f}')
        
        # IDs de ArUco objetivo
        self.target_aruco_ids = [20, 21, 22, 23]
        
        # ID del robot (ArUco 3 o 8, ajusta según tu robot)
        self.robot_aruco_id = 3  # Cambia a 8 si tu robot es el ArUco 8
        
        # Suscripciones a todos los ArUcos
        self.aruco_data = {}
        all_ids = self.target_aruco_ids + [self.robot_aruco_id]
        for aruco_id in all_ids:
            topic = f'/overhead_camera/aruco_{aruco_id}'
            self.create_subscription(
                String, 
                topic, 
                lambda msg, aid=aruco_id: self.aruco_callback(msg, aid), 
                10
            )
        
        # Publicador de comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Estado del navegador
        self.current_goal_id = None
        self.state = 'SELECTING_TARGET'  # Estados: SELECTING_TARGET, NAVIGATING, REACHED
        self.last_update_time = time.time()
        
        # Timer de control (10 Hz)
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('ArUco Navigator iniciado')
        self.get_logger().info(f'Robot ArUco ID: {self.robot_aruco_id}')
        self.get_logger().info(f'Objetivos: {self.target_aruco_ids}')
    
    def aruco_callback(self, msg: String, aruco_id: int):
        """Callback para recibir datos de ArUco."""
        try:
            data = json.loads(msg.data)
            px = float(data.get('px', 0.0))
            py = float(data.get('py', 0.0))
            orientation = float(data.get('orientation', 0.0))
            
            self.aruco_data[aruco_id] = {
                'px': px,
                'py': py,
                'x': px * self.px_to_m_x,  # Convertir a metros
                'y': py * self.px_to_m_y,
                'orientation': orientation,
                'timestamp': time.time()
            }
        except Exception as e:
            self.get_logger().warn(f'Error procesando ArUco {aruco_id}: {e}')
    
    def get_aruco_position(self, aruco_id: int) -> Optional[Dict]:
        """Obtiene la posición de un ArUco si fue visto recientemente."""
        if aruco_id not in self.aruco_data:
            return None
        
        data = self.aruco_data[aruco_id]
        # Solo usar datos recientes (últimos 0.5 segundos)
        if time.time() - data['timestamp'] > 0.5:
            return None
        
        return data
    
    def select_random_target(self):
        """Selecciona un objetivo aleatorio entre los ArUcos disponibles."""
        # Filtrar solo los ArUcos que están visibles
        available_targets = [
            aid for aid in self.target_aruco_ids 
            if self.get_aruco_position(aid) is not None
        ]
        
        if not available_targets:
            self.get_logger().warn('No hay objetivos visibles')
            return None
        
        # Evitar seleccionar el mismo objetivo inmediatamente
        if len(available_targets) > 1 and self.current_goal_id in available_targets:
            available_targets.remove(self.current_goal_id)
        
        selected = random.choice(available_targets)
        self.get_logger().info(f'🎯 Nuevo objetivo seleccionado: ArUco {selected}')
        return selected
    
    def calculate_control(self, robot_pos: Dict, goal_pos: Dict) -> Tuple[float, float]:
        """
        Calcula velocidades lineales y angulares para ir del robot al objetivo.
        Retorna: (linear_velocity, angular_velocity)
        """
        # Diferencias en posición
        dx = goal_pos['x'] - robot_pos['x']
        dy = goal_pos['y'] - robot_pos['y']
        
        # Distancia al objetivo
        distance = math.sqrt(dx**2 + dy**2)
        
        # Ángulo hacia el objetivo (en radianes)
        angle_to_goal = math.atan2(dy, dx)
        
        # Orientación actual del robot (convertir de grados a radianes)
        robot_orientation = math.radians(robot_pos['orientation'])
        
        # Diferencia angular
        angle_error = angle_to_goal - robot_orientation
        
        # Normalizar ángulo entre -π y π
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Control proporcional
        if distance < self.goal_tolerance:
            # Objetivo alcanzado
            return 0.0, 0.0
        
        # Velocidad lineal proporcional a la distancia
        linear_vel = min(distance * 0.8, self.max_linear_speed)
        
        # Si el ángulo de error es grande, reducir velocidad lineal y priorizar giro
        if abs(angle_error) > math.radians(30):  # Más de 30 grados
            linear_vel *= 0.3
        
        # Velocidad angular proporcional al error angular
        angular_vel = angle_error * 2.0
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
        
        return linear_vel, angular_vel
    
    def control_loop(self):
        """Bucle principal de control."""
        
        if self.state == 'SELECTING_TARGET':
            # Seleccionar nuevo objetivo
            self.current_goal_id = self.select_random_target()
            if self.current_goal_id is not None:
                self.state = 'NAVIGATING'
                self.last_update_time = time.time()
        
        elif self.state == 'NAVIGATING':
            # Obtener posición del robot
            robot_pos = self.get_aruco_position(self.robot_aruco_id)
            if robot_pos is None:
                self.get_logger().warn(f'Robot (ArUco {self.robot_aruco_id}) no visible')
                self.stop_robot()
                return
            
            # Obtener posición del objetivo
            goal_pos = self.get_aruco_position(self.current_goal_id)
            if goal_pos is None:
                self.get_logger().warn(f'Objetivo (ArUco {self.current_goal_id}) no visible')
                self.stop_robot()
                self.state = 'SELECTING_TARGET'
                return
            
            # Calcular control
            linear_vel, angular_vel = self.calculate_control(robot_pos, goal_pos)
            
            # Calcular distancia al objetivo
            dx = goal_pos['x'] - robot_pos['x']
            dy = goal_pos['y'] - robot_pos['y']
            distance = math.sqrt(dx**2 + dy**2)
            
            # Publicar comando de velocidad
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)
            
            # Mostrar progreso cada segundo
            if time.time() - self.last_update_time > 1.0:
                self.get_logger().info(
                    f'Navegando a ArUco {self.current_goal_id} | '
                    f'Distancia: {distance:.2f}m | '
                    f'Vel: lin={linear_vel:.2f} ang={angular_vel:.2f}'
                )
                self.last_update_time = time.time()
            
            # Verificar si llegamos al objetivo
            if distance < self.goal_tolerance:
                self.get_logger().info(f'✅ Objetivo ArUco {self.current_goal_id} alcanzado!')
                self.stop_robot()
                self.state = 'REACHED'
                self.last_update_time = time.time()
        
        elif self.state == 'REACHED':
            # Esperar un momento antes de seleccionar nuevo objetivo
            if time.time() - self.last_update_time > 2.0:
                self.state = 'SELECTING_TARGET'
    
    def stop_robot(self):
        """Detiene el robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    navigator = ArucoNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Deteniendo navegador...')
        navigator.stop_robot()
    
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

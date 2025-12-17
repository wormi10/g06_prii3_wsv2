#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import random
import time


class VisionGuidedNavigation(Node):
    def __init__(self):
        super().__init__('vision_guided_navigation')
        
        # Parametros configurables
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('distance_tolerance', 0.15)
        self.declare_parameter('angle_tolerance', 0.08)
        self.declare_parameter('wait_time', 5.0)  # Cambiado a 5 segundos en cada objetivo
        self.declare_parameter('start_delay', 10.0)  # Nuevo: 10 segundos para esperar RViz
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.wait_time = self.get_parameter('wait_time').value
        self.start_delay = self.get_parameter('start_delay').value
        
        # Publisher para comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF2 para escuchar transformaciones
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Estado del nodo
        self.target_aruco = None
        self.state = 'WAITING_FOR_RVIZ'  # Nuevo estado: esperando RViz
        self.wait_start_time = None
        self.start_time = None  # Tiempo de inicio del nodo
        self.visited_arucos = []
        
        # Lista de ArUcos detectados
        self.detected_arucos = []
        self.available_arucos = []
        self.initialization_complete = False
        
        # Frames de referencia
        self.reference_frame = 'aruco_2'  # Origen del sistema (esquina del tablero)
        self.robot_frame = 'aruco_3'      # ArUco del robot
        
        # Timer para inicializacion
        self.init_timer = self.create_timer(0.5, self.initialize_arucos)
        
        # Timer para control de navegacion
        self.navigation_timer = None
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("NODO DE NAVEGACION GUIADA POR VISION INICIADO")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Frame de referencia (origen): {self.reference_frame}")
        self.get_logger().info(f"Frame del robot: {self.robot_frame}")
        self.get_logger().info(f"Velocidad lineal: {self.linear_speed} m/s")
        self.get_logger().info(f"Velocidad angular: {self.angular_speed} rad/s")
        self.get_logger().info(f"Tolerancia distancia: {self.distance_tolerance} m")
        self.get_logger().info(f"Tolerancia angulo: {math.degrees(self.angle_tolerance):.1f} grados")
        self.get_logger().info(f"Tiempo de espera en objetivos: {self.wait_time}s")
        self.get_logger().info(f"Esperando {self.start_delay}s para RViz antes de iniciar...")
        self.get_logger().info("Detectando ArUcos disponibles en el tablero...")
        self.get_logger().info("=" * 70)
        
        # Registrar tiempo de inicio
        self.start_time = time.time()
        
    def initialize_arucos(self):
        """Detecta automaticamente los ArUcos disponibles al inicio"""
        if self.initialization_complete:
            return
        
        # Verificar que exista el frame de referencia (ArUco 2)
        try:
            self.tf_buffer.lookup_transform(
                'overhead_camera_link',
                self.reference_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except:
            self.get_logger().info(
                "Esperando ArUco 2 (origen del tablero)...",
                throttle_duration_sec=2.0
            )
            return
        
        # Verificar que exista el frame del robot (ArUco 3)
        try:
            self.tf_buffer.lookup_transform(
                'overhead_camera_link',
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except:
            self.get_logger().info(
                "Esperando ArUco 3 (robot)...",
                throttle_duration_sec=2.0
            )
            return
        
        # Buscar todos los ArUcos disponibles (IDs del 0 al 50)
        detected = []
        for aruco_id in range(51):
            if aruco_id == 2 or aruco_id == 3:  # Excluir origen y robot
                continue
            
            frame_name = f'aruco_{aruco_id}'
            try:
                self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    frame_name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                detected.append(aruco_id)
            except:
                pass
        
        if len(detected) > 0:
            self.detected_arucos = detected
            self.available_arucos = detected.copy()
            self.initialization_complete = True
            
            # Detener timer de inicializacion
            self.init_timer.cancel()
            
            # Iniciar timer de navegacion
            self.navigation_timer = self.create_timer(0.1, self.navigation_loop)
            
            self.get_logger().info("=" * 70)
            self.get_logger().info("INICIALIZACION COMPLETADA")
            self.get_logger().info(f"ArUcos detectados (excluyendo 2 y 3): {self.detected_arucos}")
            self.get_logger().info(f"Total de objetivos disponibles: {len(self.detected_arucos)}")
            self.get_logger().info(f"Esperando {self.start_delay}s antes de iniciar navegacion...")
            self.get_logger().info("=" * 70)
        else:
            self.get_logger().info(
                "Buscando ArUcos en el tablero...",
                throttle_duration_sec=2.0
            )
    
    def get_transform(self, target_frame, source_frame):
        """Obtiene la transformacion entre dos frames"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return transform
        except TransformException:
            return None
            
    def calculate_distance(self, transform):
        """Calcula la distancia euclidiana desde la transformacion"""
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        return math.sqrt(x**2 + y**2)
    
    def calculate_angle(self, transform):
        """Calcula el angulo hacia el objetivo"""
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        return math.atan2(y, x)
    
    def select_random_aruco(self):
        """Selecciona un ArUco aleatorio de los disponibles (excluyendo 2 y 3)"""
        if not self.detected_arucos:
            self.get_logger().warn('No hay ArUcos disponibles (excluyendo 2 y 3)')
            return None
        
        # Verificar cuales ArUcos detectados siguen siendo visibles
        currently_visible = []
        for aruco_id in self.detected_arucos:
            # Excluir siempre los ArUcos 2 y 3
            if aruco_id == 2 or aruco_id == 3:
                continue
                
            target_frame = f'aruco_{aruco_id}'
            if self.get_transform(self.reference_frame, target_frame):
                currently_visible.append(aruco_id)
        
        if not currently_visible:
            self.get_logger().warn(
                'No se detectan ArUcos en el tablero (excluyendo 2 y 3)',
                throttle_duration_sec=3.0
            )
            return None
        
        # Si ya visitamos todos, reiniciar lista
        unvisited = [a for a in currently_visible if a not in self.visited_arucos]
        
        if not unvisited:
            self.visited_arucos = []
            unvisited = currently_visible
            self.get_logger().info("=" * 70)
            self.get_logger().info("TODOS LOS ARUCOS VISITADOS, REINICIANDO CICLO")
            self.get_logger().info("=" * 70)
        
        selected = random.choice(unvisited)
        self.visited_arucos.append(selected)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"NUEVO OBJETIVO: ArUco {selected}")
        self.get_logger().info(f"ArUcos visitados: {self.visited_arucos}")
        self.get_logger().info(f"ArUcos pendientes: {[a for a in currently_visible if a not in self.visited_arucos]}")
        self.get_logger().info("=" * 70)
        
        return selected
    
    def publish_velocity(self, linear=0.0, angular=0.0):
        """Publica comando de velocidad"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
    
    def stop_robot(self):
        """Detiene el robot completamente"""
        self.publish_velocity(0.0, 0.0)
    
    def navigation_loop(self):
        """Loop principal de navegacion - MOVIMIENTOS RECTOS"""
        
        if not self.initialization_complete:
            return
        
        # ==================================================================
        # Estado WAITING_FOR_RVIZ: Esperar 10 segundos antes de iniciar
        # ==================================================================
        if self.state == 'WAITING_FOR_RVIZ':
            elapsed = time.time() - self.start_time
            remaining = self.start_delay - elapsed
            
            if remaining > 0:
                if int(remaining) != int(remaining + 0.1):  # Mostrar solo una vez por segundo
                    self.get_logger().info(f"Iniciando navegacion en {int(remaining + 1)}s...")
                self.stop_robot()  # Asegurarse de que el robot no se mueva
                return
            else:
                self.get_logger().info("=" * 70)
                self.get_logger().info("TIEMPO DE ESPERA FINALIZADO - INICIANDO NAVEGACION")
                self.get_logger().info("=" * 70)
                self.state = 'IDLE'
        
        # Estado IDLE: Seleccionar nuevo objetivo
        if self.state == 'IDLE':
            self.target_aruco = self.select_random_aruco()
            if self.target_aruco is not None:
                self.state = 'ROTATING'
                self.get_logger().info(f"Iniciando navegacion hacia ArUco {self.target_aruco}")
            return
        
        # Verificar que tenemos un objetivo
        if self.target_aruco is None:
            self.stop_robot()
            self.state = 'IDLE'
            return
        
        # Estado WAITING: Esperar 5 segundos en el objetivo
        if self.state == 'WAITING':
            if self.wait_start_time is None:
                self.wait_start_time = time.time()
                self.get_logger().info(f"Esperando {self.wait_time}s en ArUco {self.target_aruco}")
                self.get_logger().info(f"El robot estará quieto por {self.wait_time} segundos")
            
            elapsed = time.time() - self.wait_start_time
            remaining = self.wait_time - elapsed
            
            if remaining > 0:
                # Mostrar cuenta regresiva cada segundo
                if int(remaining) != int(remaining + 0.1):
                    self.get_logger().info(f"Espera: {int(remaining + 1)}s restantes...")
                self.stop_robot()  # Mantener el robot quieto
            else:
                self.get_logger().info("Espera completada, seleccionando nuevo objetivo...")
                self.wait_start_time = None
                self.state = 'IDLE'
            return
        
        # Verificar frame de referencia (ArUco 2)
        if not self.get_transform('overhead_camera_link', self.reference_frame):
            self.get_logger().warn(
                'ArUco 2 (origen) no detectado',
                throttle_duration_sec=3.0
            )
            self.stop_robot()
            self.state = 'WAITING_FOR_RVIZ'  # Volver a estado de espera
            return
        
        # Obtener posicion del robot (ArUco 3)
        robot_transform = self.get_transform(self.reference_frame, self.robot_frame)
        if not robot_transform:
            self.get_logger().warn(
                'ArUco 3 (robot) no detectado',
                throttle_duration_sec=2.0
            )
            self.stop_robot()
            return
        
        # Obtener posicion del objetivo (excluyendo 2 y 3)
        target_frame = f'aruco_{self.target_aruco}'
        target_transform = self.get_transform(self.reference_frame, target_frame)
        
        if not target_transform:
            self.get_logger().warn(
                f'ArUco {self.target_aruco} perdido, seleccionando nuevo objetivo',
                throttle_duration_sec=2.0
            )
            self.stop_robot()
            self.state = 'IDLE'
            return
        
        # Calcular transformacion relativa: robot -> objetivo
        relative_transform = TransformStamped()
        relative_transform.transform.translation.x = (
            target_transform.transform.translation.x - 
            robot_transform.transform.translation.x
        )
        relative_transform.transform.translation.y = (
            target_transform.transform.translation.y - 
            robot_transform.transform.translation.y
        )
        
        # Calcular distancia y angulo
        distance = self.calculate_distance(relative_transform)
        angle = self.calculate_angle(relative_transform)
        
        # ==================================================================
        # Estado ROTATING: Girar hasta estar perfectamente alineado
        # ==================================================================
        if self.state == 'ROTATING':
            if abs(angle) > self.angle_tolerance:
                # Girar hacia el objetivo
                angular_vel = self.angular_speed if angle > 0 else -self.angular_speed
                self.publish_velocity(0.0, angular_vel)
                
                self.get_logger().info(
                    f"Rotando hacia ArUco {self.target_aruco} | Angulo: {math.degrees(angle):+.1f} grados",
                    throttle_duration_sec=0.5
                )
            else:
                # Perfectamente alineado
                self.get_logger().info(f"Alineado con ArUco {self.target_aruco}! Angulo: {math.degrees(angle):+.1f} grados")
                self.stop_robot()
                time.sleep(0.3)  # Pausa para estabilizar
                self.state = 'MOVING'
            return
        
        # ==================================================================
        # Estado MOVING: Avanzar en linea recta
        # ==================================================================
        if self.state == 'MOVING':
            if distance > self.distance_tolerance:
                # Verificar si nos hemos desalineado demasiado
                if abs(angle) > self.angle_tolerance * 3:
                    self.get_logger().info("Desalineacion detectada, reajustando orientacion")
                    self.stop_robot()
                    time.sleep(0.2)
                    self.state = 'ROTATING'
                    return
                
                # Avanzar recto con minima correccion angular
                # Solo aplicar una pequeña correccion si es necesario
                if abs(angle) > self.angle_tolerance:
                    angular_correction = angle * 0.3  # Correccion suave
                    angular_correction = max(-self.angular_speed/3, 
                                            min(self.angular_speed/3, angular_correction))
                else:
                    angular_correction = 0.0
                
                self.publish_velocity(self.linear_speed, angular_correction)
                
                self.get_logger().info(
                    f"Avanzando hacia ArUco {self.target_aruco} | "
                    f"Distancia: {distance:.2f}m | "
                    f"Angulo: {math.degrees(angle):+.1f} grados",
                    throttle_duration_sec=0.8
                )
            else:
                # Objetivo alcanzado
                self.get_logger().info("=" * 70)
                self.get_logger().info(f"OBJETIVO ALCANZADO: ArUco {self.target_aruco}")
                self.get_logger().info(f"Posicion alcanzada con precision: {distance:.3f}m")
                self.get_logger().info("El robot esperará 5 segundos en esta posición")
                self.get_logger().info("=" * 70)
                self.stop_robot()
                self.state = 'WAITING'
            return


def main(args=None):
    rclpy.init(args=args)
    node = VisionGuidedNavigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("=" * 70)
        node.get_logger().info("NAVEGACION DETENIDA POR EL USUARIO")
        node.get_logger().info("=" * 70)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
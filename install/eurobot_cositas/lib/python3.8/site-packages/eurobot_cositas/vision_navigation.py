#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import math
import time
from scipy.spatial.transform import Rotation as R

class VisionGuidedNavigation(Node):
    def __init__(self):
        super().__init__('vision_guided_navigation')
        
        # ==========================================
        # CONFIGURACIÓN DE RUTA Y PARÁMETROS
        # ==========================================
        # Pon aquí los ArUcos en el orden exacto que quieras:
        self.ORDEN_DESEADO = [20, 21, 22, 23] 
        
        self.linear_speed = 0.08        # Velocidad de avance
        self.angular_speed = 0.3     # Velocidad de giro constante
        self.distance_tolerance = 0.35  # Margen para detenerse ante el ArUco
        self.wait_time = 5.0            # Tiempo de espera en cada punto
        self.start_delay = 5.0          # Tiempo de espera inicial
        
        # Si el robot se desvía a la izquierda al ir recto, pon un valor negativo pequeño:
        self.drift_correction = -0.00 
        
        # ==========================================
        # INICIALIZACIÓN DE ROS 2
        # ==========================================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.target_aruco = None
        self.state = 'WAITING_FOR_RVIZ' 
        self.visited_arucos = []
        self.detected_arucos = []
        self.initialization_complete = False
        
        self.reference_frame = 'aruco_2' # El origen de coordenadas
        self.robot_frame = 'aruco_3'     # El robot
        
        self.init_timer = self.create_timer(0.5, self.initialize_arucos)
        self.navigation_timer = None
        self.start_time = time.time()
        
        self.get_logger().info("Nodo iniciado. Esperando conexión de visión...")

    def initialize_arucos(self):
        """Busca los ArUcos de la lista en el orden indicado."""
        if self.initialization_complete: return
        try:
            # Confirmar que vemos la base y el robot
            self.tf_buffer.lookup_transform(self.reference_frame, self.robot_frame, rclpy.time.Time())
            
            temp_detected = []
            for i in self.ORDEN_DESEADO:
                try:
                    self.tf_buffer.lookup_transform(
                        self.reference_frame, 
                        f'aruco_{i}', 
                        rclpy.time.Time(), 
                        timeout=rclpy.duration.Duration(seconds=0.05)
                    )
                    temp_detected.append(i)
                except:
                    continue # Si no ve uno, intenta el siguiente de la lista
            
            if temp_detected:
                self.detected_arucos = temp_detected
                self.initialization_complete = True
                self.init_timer.cancel()
                self.navigation_timer = self.create_timer(0.1, self.navigation_loop)
                self.get_logger().info(f"RUTA CARGADA: {self.detected_arucos}")
        except: return

    def get_absolute_pose(self, frame_name):
        """Obtiene (x, y, yaw) respecto al ArUco 2."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.reference_frame, 
                frame_name, 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            quat = t.transform.rotation
            yaw = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        except: return None

    def publish_vel(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.cmd_vel_pub.publish(msg)

    def navigation_loop(self):
        if self.state == 'WAITING_FOR_RVIZ':
            if (time.time() - self.start_time) > self.start_delay:
                self.state = 'IDLE'
            return

        # 1. SELECCIÓN DE OBJETIVO (ORDENADO)
        if self.state == 'IDLE':
            unvisited = [a for a in self.detected_arucos if a not in self.visited_arucos]
            if not unvisited:
                self.visited_arucos = [] # Reiniciar ciclo si se desea
                unvisited = self.detected_arucos
            
            self.target_aruco = unvisited[0]
            self.visited_arucos.append(self.target_aruco)
            self.get_logger().info(f"--- NUEVO DESTINO: ArUco {self.target_aruco} ---")
            self.state = 'START_TURN'
            return

        # 2. GIRO ÚNICO (POR TIEMPO)
        if self.state == 'START_TURN':
            robot = self.get_absolute_pose(self.robot_frame)
            target = self.get_absolute_pose(f'aruco_{self.target_aruco}')
            
            if robot and target:
                dx, dy = target[0] - robot[0], target[1] - robot[1]
                target_angle = math.atan2(dy, dx)
                # Inversión de signo solicitada para tu setup
                if self.target_aruco in [20, 21,22]:
                 angle_error = -math.atan2(math.sin(target_angle - robot[2]), math.cos(target_angle - robot[2]))
                else:
                 angle_error = math.atan2(math.sin(target_angle - robot[2]), math.cos(target_angle - robot[2]))
                duration = abs(angle_error) / self.angular_speed
                direction = 1.0 if angle_error > 0 else -1.0
                
                self.get_logger().info(f"Girando {math.degrees(angle_error):.1f}°...")
                t_end = time.time() + duration
                while time.time() < t_end:
                    self.publish_vel(0.0, self.angular_speed * direction)
                    time.sleep(0.01)
                
                self.publish_vel(0.0, 0.0)
                time.sleep(1.2) # Pausa de estabilización clave para el siguiente paso
                self.state = 'GO_STRAIGHT'
            return

        # 3. AVANCE RECTO (HASTA LLEGAR A POSICIÓN)
        if self.state == 'GO_STRAIGHT':
            robot = self.get_absolute_pose(self.robot_frame)
            target = self.get_absolute_pose(f'aruco_{self.target_aruco}')
            
            if robot and target:
                dx, dy = target[0] - robot[0], target[1] - robot[1]
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist > self.distance_tolerance:
                    # Avanza recto + compensación mecánica si es necesaria
                    if self.target_aruco in [21, 22, 23]:
                        self.drift_correction = -0.027
                    else:
                        self.drift_correction = 0.00   
                    self.publish_vel(self.linear_speed, self.drift_correction)
                else:
                    self.get_logger().info(f"¡Llegado al ArUco {self.target_aruco}!")
                    self.publish_vel(0.0, 0.0)
                    self.state = 'WAITING'
            return

        # 4. ESPERA ANTES DEL SIGUIENTE
        if self.state == 'WAITING':
            if not hasattr(self, '_w_start'): self._w_start = time.time()
            
            if (time.time() - self._w_start) > self.wait_time:
                del self._w_start
                self.state = 'IDLE' # Volver a IDLE recalcula todo para el siguiente ArUco
            else:
                self.publish_vel(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = VisionGuidedNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
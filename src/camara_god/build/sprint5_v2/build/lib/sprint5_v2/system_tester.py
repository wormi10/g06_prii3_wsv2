#!/usr/bin/env python3
"""
Script de prueba para verificar el sistema antes de la navegación completa.
Muestra información de diagnóstico y realiza pruebas básicas.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import time
import math


class SystemTester(Node):
    """Nodo para probar y diagnosticar el sistema de navegación."""
    
    def __init__(self):
        super().__init__('system_tester')
        
        # Parámetros configurables
        self.declare_parameter('camera_resolution_x', 1280)
        self.declare_parameter('camera_resolution_y', 720)
        self.declare_parameter('real_world_width', 2.0)
        self.declare_parameter('real_world_height', 1.5)
        
        # Obtener parámetros
        self.cam_res_x = self.get_parameter('camera_resolution_x').value
        self.cam_res_y = self.get_parameter('camera_resolution_y').value
        self.real_width = self.get_parameter('real_world_width').value
        self.real_height = self.get_parameter('real_world_height').value
        
        # Factor de conversión
        self.px_to_m_x = self.real_width / self.cam_res_x
        self.px_to_m_y = self.real_height / self.cam_res_y
        
        # Datos de ArUcos
        self.target_ids = [20, 21, 22, 23]
        self.robot_ids = [3, 8]
        self.aruco_data = {}
        
        # Suscripciones
        all_ids = self.target_ids + self.robot_ids
        for aruco_id in all_ids:
            topic = f'/overhead_camera/aruco_{aruco_id}'
            self.create_subscription(
                String, 
                topic, 
                lambda msg, aid=aruco_id: self.aruco_callback(msg, aid), 
                10
            )
        
        # Publicador de velocidad (para prueba)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('')
        self.get_logger().info('═' * 80)
        self.get_logger().info('  SISTEMA DE PRUEBA - NAVEGACIÓN ARUCO')
        self.get_logger().info('═' * 80)
        self.get_logger().info('')
        
        # Timer para diagnóstico
        self.create_timer(3.0, self.run_diagnostics)
        self.test_count = 0
    
    def aruco_callback(self, msg: String, aruco_id: int):
        """Callback para ArUcos."""
        try:
            data = json.loads(msg.data)
            px = float(data.get('px', 0.0))
            py = float(data.get('py', 0.0))
            orientation = float(data.get('orientation', 0.0))
            
            self.aruco_data[aruco_id] = {
                'px': px,
                'py': py,
                'x': px * self.px_to_m_x,
                'y': py * self.px_to_m_y,
                'orientation': orientation,
                'timestamp': time.time()
            }
        except Exception as e:
            self.get_logger().error(f'Error procesando ArUco {aruco_id}: {e}')
    
    def run_diagnostics(self):
        """Ejecutar pruebas de diagnóstico."""
        self.test_count += 1
        
        self.get_logger().info('')
        self.get_logger().info('─' * 80)
        self.get_logger().info(f'  DIAGNÓSTICO #{self.test_count}')
        self.get_logger().info('─' * 80)
        
        # Test 1: Parámetros de conversión
        self.get_logger().info('')
        self.get_logger().info('✓ TEST 1: Parámetros de conversión')
        self.get_logger().info(f'  Resolución cámara: {self.cam_res_x}x{self.cam_res_y} px')
        self.get_logger().info(f'  Área real: {self.real_width:.2f}x{self.real_height:.2f} m')
        self.get_logger().info(f'  Escala X: {self.px_to_m_x:.6f} m/px')
        self.get_logger().info(f'  Escala Y: {self.px_to_m_y:.6f} m/px')
        
        # Test 2: Detectar ArUcos visibles
        self.get_logger().info('')
        self.get_logger().info('✓ TEST 2: ArUcos detectados')
        
        now = time.time()
        visible_targets = []
        visible_robots = []
        
        for aruco_id, data in self.aruco_data.items():
            age = now - data['timestamp']
            status = '✓ ACTIVO' if age < 0.5 else '⚠ ANTIGUO'
            
            if aruco_id in self.target_ids:
                visible_targets.append(aruco_id)
                category = 'OBJETIVO'
            else:
                visible_robots.append(aruco_id)
                category = 'ROBOT'
            
            self.get_logger().info(
                f'  ArUco {aruco_id:2d} ({category:8s}): '
                f'px=({data["px"]:7.2f}, {data["py"]:7.2f}) → '
                f'm=({data["x"]:.3f}, {data["y"]:.3f}) | '
                f'ori={data["orientation"]:6.1f}° | '
                f'{status} ({age:.1f}s)'
            )
        
        if not self.aruco_data:
            self.get_logger().warn('  ⚠️  NO SE DETECTAN ARUCOS - Verifica la cámara')
        
        # Test 3: Estado de objetivos
        self.get_logger().info('')
        self.get_logger().info('✓ TEST 3: Estado de objetivos')
        
        recent_targets = [aid for aid in visible_targets 
                         if now - self.aruco_data[aid]['timestamp'] < 0.5]
        
        self.get_logger().info(f'  Objetivos disponibles: {len(recent_targets)}/4')
        
        if recent_targets:
            self.get_logger().info(f'  IDs disponibles: {recent_targets}')
        else:
            self.get_logger().warn('  ⚠️  NO HAY OBJETIVOS DISPONIBLES')
        
        # Test 4: Detectar robot
        self.get_logger().info('')
        self.get_logger().info('✓ TEST 4: Detección del robot')
        
        recent_robots = [aid for aid in visible_robots 
                        if now - self.aruco_data[aid]['timestamp'] < 0.5]
        
        if recent_robots:
            self.get_logger().info(f'  Robot detectado: ArUco {recent_robots[0]}')
            if len(recent_robots) > 1:
                self.get_logger().warn(f'  ⚠️  Múltiples robots detectados: {recent_robots}')
                self.get_logger().warn('  Asegúrate de configurar el ID correcto en aruco_navigator.py')
        else:
            self.get_logger().warn('  ⚠️  NO SE DETECTA ROBOT (ArUcos 3 u 8)')
        
        # Test 5: Calcular distancias entre objetivos
        if len(recent_targets) >= 2:
            self.get_logger().info('')
            self.get_logger().info('✓ TEST 5: Distancias entre objetivos')
            
            for i in range(len(recent_targets)):
                for j in range(i + 1, min(i + 2, len(recent_targets))):  # Solo algunos pares
                    id1, id2 = recent_targets[i], recent_targets[j]
                    pos1 = self.aruco_data[id1]
                    pos2 = self.aruco_data[id2]
                    
                    dx = pos2['x'] - pos1['x']
                    dy = pos2['y'] - pos1['y']
                    dist = math.sqrt(dx**2 + dy**2)
                    
                    self.get_logger().info(
                        f'  ArUco {id1} → ArUco {id2}: {dist:.3f} m'
                    )
        
        # Test 6: Simulación de control
        if recent_robots and recent_targets:
            self.get_logger().info('')
            self.get_logger().info('✓ TEST 6: Simulación de control')
            
            robot_id = recent_robots[0]
            target_id = recent_targets[0]
            
            robot_pos = self.aruco_data[robot_id]
            target_pos = self.aruco_data[target_id]
            
            dx = target_pos['x'] - robot_pos['x']
            dy = target_pos['y'] - robot_pos['y']
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_goal = math.atan2(dy, dx)
            
            robot_orientation = math.radians(robot_pos['orientation'])
            angle_error = angle_to_goal - robot_orientation
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            self.get_logger().info(f'  Robot (ArUco {robot_id}) → Objetivo (ArUco {target_id})')
            self.get_logger().info(f'  Distancia: {distance:.3f} m')
            self.get_logger().info(f'  Error angular: {math.degrees(angle_error):.1f}°')
            
            # Calcular velocidades sugeridas
            linear_vel = min(distance * 0.8, 0.5)
            angular_vel = angle_error * 2.0
            angular_vel = max(-1.0, min(1.0, angular_vel))
            
            self.get_logger().info(f'  Velocidad lineal sugerida: {linear_vel:.2f} m/s')
            self.get_logger().info(f'  Velocidad angular sugerida: {angular_vel:.2f} rad/s')
        
        # Resumen final
        self.get_logger().info('')
        self.get_logger().info('─' * 80)
        self.get_logger().info('  RESUMEN')
        self.get_logger().info('─' * 80)
        
        issues = []
        
        if not self.aruco_data:
            issues.append('⚠️  No se detectan ArUcos')
        if not recent_targets:
            issues.append('⚠️  No hay objetivos disponibles (ArUcos 20-23)')
        if not recent_robots:
            issues.append('⚠️  No se detecta robot (ArUcos 3 u 8)')
        
        if issues:
            self.get_logger().warn('  PROBLEMAS DETECTADOS:')
            for issue in issues:
                self.get_logger().warn(f'  {issue}')
        else:
            self.get_logger().info('  ✓ Sistema listo para navegación')
            self.get_logger().info('  Puedes ejecutar: ros2 run tu_paquete aruco_navigator')
        
        self.get_logger().info('')
        self.get_logger().info('═' * 80)


def main(args=None):
    rclpy.init(args=args)
    tester = SystemTester()
    
    print("\n" + "="*80)
    print("  PRUEBA DEL SISTEMA - Presiona Ctrl+C para salir")
    print("="*80 + "\n")
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n\n✓ Prueba finalizada\n")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

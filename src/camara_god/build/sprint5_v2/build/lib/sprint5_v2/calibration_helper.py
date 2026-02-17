#!/usr/bin/env python3
"""
Script de calibración para convertir coordenadas de píxeles a metros.
Ayuda a determinar los parámetros real_world_width y real_world_height.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import sys


class CalibrationHelper(Node):
    """
    Nodo auxiliar para calibrar la conversión píxeles -> metros.
    
    USO:
    1. Coloca dos marcadores ArUco en el área
    2. Mide la distancia física entre ellos con una cinta métrica
    3. Ejecuta este script y sigue las instrucciones
    """
    
    def __init__(self):
        super().__init__('calibration_helper')
        
        # Suscribirse a todos los ArUcos
        self.aruco_ids = [3, 8, 20, 21, 22, 23]
        self.aruco_positions = {}
        
        for aruco_id in self.aruco_ids:
            topic = f'/overhead_camera/aruco_{aruco_id}'
            self.create_subscription(
                String, 
                topic, 
                lambda msg, aid=aruco_id: self.aruco_callback(msg, aid), 
                10
            )
        
        self.get_logger().info('═' * 70)
        self.get_logger().info('  ASISTENTE DE CALIBRACIÓN - CÁMARA CENITAL')
        self.get_logger().info('═' * 70)
        self.get_logger().info('')
        self.get_logger().info('Esperando datos de ArUcos...')
        
        # Timer para mostrar posiciones
        self.create_timer(2.0, self.show_positions)
    
    def aruco_callback(self, msg: String, aruco_id: int):
        """Recibir posiciones de ArUcos."""
        try:
            data = json.loads(msg.data)
            px = float(data.get('px', 0.0))
            py = float(data.get('py', 0.0))
            
            self.aruco_positions[aruco_id] = {'px': px, 'py': py}
        except Exception as e:
            pass
    
    def show_positions(self):
        """Mostrar posiciones actuales y ayudar con cálculos."""
        if not self.aruco_positions:
            self.get_logger().warn('⚠️  No se detectan ArUcos. Verifica la cámara.')
            return
        
        self.get_logger().info('')
        self.get_logger().info('─' * 70)
        self.get_logger().info('  POSICIONES ACTUALES (en píxeles)')
        self.get_logger().info('─' * 70)
        
        for aruco_id in sorted(self.aruco_positions.keys()):
            pos = self.aruco_positions[aruco_id]
            self.get_logger().info(f'  ArUco {aruco_id:2d}: px={pos["px"]:7.2f}  py={pos["py"]:7.2f}')
        
        # Si hay al menos 2 ArUcos, calcular distancias
        if len(self.aruco_positions) >= 2:
            self.get_logger().info('')
            self.get_logger().info('─' * 70)
            self.get_logger().info('  DISTANCIAS ENTRE ARUCOS (en píxeles)')
            self.get_logger().info('─' * 70)
            
            ids = sorted(self.aruco_positions.keys())
            for i in range(len(ids)):
                for j in range(i + 1, len(ids)):
                    id1, id2 = ids[i], ids[j]
                    pos1 = self.aruco_positions[id1]
                    pos2 = self.aruco_positions[id2]
                    
                    dx = pos2['px'] - pos1['px']
                    dy = pos2['py'] - pos1['py']
                    dist = math.sqrt(dx**2 + dy**2)
                    
                    self.get_logger().info(
                        f'  ArUco {id1:2d} ↔ ArUco {id2:2d}: {dist:7.2f} px  '
                        f'(Δx={dx:7.2f}, Δy={dy:7.2f})'
                    )
            
            self.get_logger().info('')
            self.get_logger().info('═' * 70)
            self.get_logger().info('  INSTRUCCIONES DE CALIBRACIÓN')
            self.get_logger().info('═' * 70)
            self.get_logger().info('')
            self.get_logger().info('  1. Elige dos ArUcos de la lista anterior')
            self.get_logger().info('  2. Mide la distancia física entre ellos (en metros)')
            self.get_logger().info('  3. Calcula la escala:')
            self.get_logger().info('')
            self.get_logger().info('     escala (m/px) = distancia_real (m) / distancia_pixels (px)')
            self.get_logger().info('')
            self.get_logger().info('  Ejemplo:')
            self.get_logger().info('     - Distancia en píxeles: 500 px')
            self.get_logger().info('     - Distancia real medida: 1.5 metros')
            self.get_logger().info('     - Escala = 1.5 / 500 = 0.003 m/px')
            self.get_logger().info('')
            self.get_logger().info('  4. Calcula las dimensiones del área:')
            self.get_logger().info('')
            self.get_logger().info('     real_world_width  = camera_resolution_x × escala')
            self.get_logger().info('     real_world_height = camera_resolution_y × escala')
            self.get_logger().info('')
            self.get_logger().info('  5. Actualiza aruco_nav_params.yaml con estos valores')
            self.get_logger().info('')
            self.get_logger().info('═' * 70)
            
            # Ejemplo de cálculo automático
            # Asumiendo resolución común de 1280x720
            ids_list = list(ids)
            if len(ids_list) >= 2:
                id1, id2 = ids_list[0], ids_list[1]
                pos1 = self.aruco_positions[id1]
                pos2 = self.aruco_positions[id2]
                
                dx = pos2['px'] - pos1['px']
                dy = pos2['py'] - pos1['py']
                dist_px = math.sqrt(dx**2 + dy**2)
                
                self.get_logger().info('')
                self.get_logger().info('  📐 CALCULADORA RÁPIDA:')
                self.get_logger().info('')
                self.get_logger().info(f'  Si la distancia entre ArUco {id1} y ArUco {id2} es:')
                self.get_logger().info(f'  - {dist_px:.2f} píxeles en la cámara')
                self.get_logger().info('')
                
                for real_dist in [0.5, 1.0, 1.5, 2.0]:
                    scale = real_dist / dist_px
                    width_1280 = 1280 * scale
                    height_720 = 720 * scale
                    
                    self.get_logger().info(f'  Si la distancia real es {real_dist} m:')
                    self.get_logger().info(f'    → escala = {scale:.6f} m/px')
                    self.get_logger().info(f'    → Para 1280x720: ancho={width_1280:.3f}m alto={height_720:.3f}m')
                    self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    helper = CalibrationHelper()
    
    print("\n" + "="*70)
    print("  ASISTENTE DE CALIBRACIÓN - Presiona Ctrl+C para salir")
    print("="*70 + "\n")
    
    try:
        rclpy.spin(helper)
    except KeyboardInterrupt:
        print("\n\n✓ Calibración finalizada\n")
    
    helper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

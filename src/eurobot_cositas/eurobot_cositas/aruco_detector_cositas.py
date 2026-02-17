#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import os
from ament_index_python.packages import get_package_share_directory

class TemplateDetectorNode(Node):
    def __init__(self):
        super().__init__('template_detector_node')
        
        # ============================================================
        # PARÁMETROS
        # ============================================================
        self.declare_parameter('camera_topic', '/overhead_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/overhead_camera/camera_info')
        self.declare_parameter('camera_frame', 'overhead_camera_link')
        self.declare_parameter('world_frame', 'world')
        
        # Template Matching
        self.declare_parameter('template_name', 'template.png')
        self.declare_parameter('match_threshold', 0.6)  # Ajustado para mejor detección
        self.declare_parameter('num_targets', 3)  # Solo 3 cuadrados del centro
        
        # Región de interés (para ignorar los lados)
        self.declare_parameter('roi_margin_percent', 15.0)  # % de margen a ignorar
        
        # Obtener parámetros
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        
        self.template_name = self.get_parameter('template_name').value
        self.match_threshold = self.get_parameter('match_threshold').value
        self.num_targets = self.get_parameter('num_targets').value
        self.roi_margin = self.get_parameter('roi_margin_percent').value / 100.0
        
        # Variables
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        self.template = None
        self.detected_targets = {}
        
        # Bridge
        self.bridge = CvBridge()
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/template/detected_image', 10)
        
        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10)
        
        # Timer para imprimir posiciones
        self.print_timer = self.create_timer(5.0, self.print_positions_callback)
        
        # Cargar template
        self.load_template()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Template Detector Node iniciado")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Cámara: {self.camera_topic}")
        self.get_logger().info(f"Template: {self.template_name}")
        self.get_logger().info(f"Umbral: {self.match_threshold}")
        self.get_logger().info(f"Objetivos: {self.num_targets}")
        self.get_logger().info(f"Margen ROI: {self.roi_margin*100:.1f}%")
        self.get_logger().info("=" * 60)
    
    def load_template(self):
        """Carga el template desde el package"""
        try:
            pkg_dir = get_package_share_directory('eurobot_cositas')
            template_path = os.path.join(pkg_dir, 'templates', self.template_name)
            
            self.template = cv2.imread(template_path)
            
            if self.template is not None:
                h, w = self.template.shape[:2]
                self.get_logger().info(f"Template cargado: {template_path} ({w}x{h}px)")
            else:
                self.get_logger().error(f" No se pudo cargar: {template_path}")
        except Exception as e:
            self.get_logger().error(f"Error al cargar template: {e}")
    
    def camera_info_callback(self, msg):
        """Callback para recibir parámetros de cámara"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Parámetros de cámara recibidos")
    
    def get_roi_mask(self, image_shape):
        """
        Crea una máscara para ignorar los lados de la imagen
        Retorna solo la región central
        """
        h, w = image_shape[:2]
        
        # Calcular márgenes
        margin_x = int(w * self.roi_margin)
        margin_y = int(h * self.roi_margin)
        
        # Crear máscara (región central = blanco, lados = negro)
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[margin_y:h-margin_y, margin_x:w-margin_x] = 255
        
        return mask, (margin_x, margin_y, w-margin_x, h-margin_y)
    
    def detect_targets_center_only(self, image, template):
        """
        Detecta objetivos usando template matching
        SOLO en la región central (ignora los lados)
        """
        if template is None:
            return []
        
        # Convertir a escala de grises
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        
        th, tw = gray_template.shape
        
        # Obtener máscara de ROI
        roi_mask, roi_bbox = self.get_roi_mask(image.shape)
        
        # Template matching
        result = cv2.matchTemplate(gray_image, gray_template, cv2.TM_CCOEFF_NORMED)
        
        # Encontrar coincidencias por encima del umbral
        locations = np.where(result >= self.match_threshold)
        
        # Convertir a lista de detecciones
        detections = []
        for pt in zip(*locations[::-1]):
            center_x = pt[0] + tw // 2
            center_y = pt[1] + th // 2
            confidence = result[pt[1], pt[0]]
            
            # FILTRAR: Solo aceptar detecciones dentro de la ROI
            if roi_mask[center_y, center_x] == 255:
                detections.append((center_x, center_y, tw, th, confidence))
        
        # Eliminar duplicados
        unique_detections = self.remove_duplicates(detections)
        
        # Limitar al número de objetivos deseado
        # Ordenar por confianza y tomar los mejores
        unique_detections = sorted(unique_detections, key=lambda x: x[4], reverse=True)
        unique_detections = unique_detections[:self.num_targets]
        
        # Ordenar por posición X (de izquierda a derecha)
        unique_detections = sorted(unique_detections, key=lambda x: x[0])
        
        return unique_detections, roi_bbox
    
    def remove_duplicates(self, detections, min_distance_ratio=0.5):
        """
        Elimina detecciones duplicadas
        """
        if not detections:
            return []
        
        # Ordenar por confianza
        detections = sorted(detections, key=lambda x: x[4], reverse=True)
        
        keep = []
        for det in detections:
            cx, cy, w, h, conf = det
            
            is_duplicate = False
            for kept in keep:
                kcx, kcy, kw, _, _ = kept
                dist = np.sqrt((cx - kcx)**2 + (cy - kcy)**2)
                
                # Si está muy cerca, es duplicado
                if dist < w * min_distance_ratio:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                keep.append(det)
        
        return keep
    
    def estimate_position_3d(self, pixel_x, pixel_y):
        """
        Estima posición 3D (asumiendo Z=0, en el plano del suelo)
        """
        if self.camera_matrix is None:
            return None
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Altura de cámara (ajustar según tu setup)
        camera_height = 2.5  # metros
        
        # Proyección inversa
        x = (pixel_x - cx) * camera_height / fx
        y = (pixel_y - cy) * camera_height / fy
        z = 0.0
        
        return (x, y, z)
    
    def publish_target_tf(self, target_id, position, timestamp):
        """Publica TF para un objetivo detectado"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = f"target_{target_id}"
        
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        
        # Sin rotación (identidad)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def print_positions_callback(self):
        
        if not self.detected_targets:
            return
        
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"OBJETIVOS DETECTADOS: {len(self.detected_targets)}")
        self.get_logger().info("=" * 80)
        
        for target_id, (pos, confidence) in sorted(self.detected_targets.items()):
            x, y, z = pos
            self.get_logger().info(
                f"Target {target_id} | Conf: {confidence:.3f} | X={x:+7.3f}m  Y={y:+7.3f}m  Z={z:+7.3f}m"
            )
        
        self.get_logger().info("=" * 80)
    
    def image_callback(self, msg):
        """Callback principal para procesar imágenes"""
        if not self.camera_info_received:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return
        
        
        if self.template is not None:
            detections, roi_bbox = self.detect_targets_center_only(cv_image, self.template)
            
            # Dibujar ROI (región de búsqueda)
            x1, y1, x2, y2 = roi_bbox
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(cv_image, "ROI de busqueda", (x1, y1-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            self.detected_targets.clear()
            
            for idx, (cx, cy, w, h, confidence) in enumerate(detections):
                position_3d = self.estimate_position_3d(cx, cy)
                
                if position_3d:
                    target_id = idx
                    self.detected_targets[target_id] = (position_3d, confidence)
                    
                    # Publicar TF
                    self.publish_target_tf(target_id, position_3d, msg.header.stamp)
                    
                    # Dibujar en imagen (VERDE para objetivos)
                    top_left = (int(cx - w//2), int(cy - h//2))
                    bottom_right = (int(cx + w//2), int(cy + h//2))
                    
                    # Rectángulo verde grueso
                    cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 3)
                    
                    # Texto con fondo
                    target_text = f"TARGET {target_id} ({confidence:.2f})"
                    text_size = cv2.getTextSize(target_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                    text_bg_top_left = (top_left[0], top_left[1] - text_size[1] - 10)
                    text_bg_bottom_right = (top_left[0] + text_size[0], top_left[1])
                    
                    cv2.rectangle(cv_image, text_bg_top_left, text_bg_bottom_right, (0, 255, 0), -1)
                    cv2.putText(cv_image, target_text, (top_left[0], top_left[1] - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                    
                    # Centro con cruz
                    cv2.drawMarker(cv_image, (cx, cy), (0, 255, 0), 
                                  cv2.MARKER_CROSS, 20, 3)
                    
                    # Coordenadas 3D
                    coord_text = f"({position_3d[0]:.2f}, {position_3d[1]:.2f})m"
                    cv2.putText(cv_image, coord_text, (int(cx) + 15, int(cy)),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            if detections:
                self.get_logger().info(
                    f"Detectados {len(detections)} objetivos",
                    throttle_duration_sec=2.0
                )
        
        
        try:
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            image_msg.header = msg.header
            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error al publicar imagen: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TemplateDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
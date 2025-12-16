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

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # Declarar parámetros
        self.declare_parameter('camera_topic', '/overhead_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/overhead_camera/camera_info')
        self.declare_parameter('camera_frame', 'overhead_camera_link')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.1)
        
        # Obtener parámetros
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.aruco_dict_type = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        
        # Variables para parámetros de cámara
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # Inicializar CvBridge
        self.bridge = CvBridge()
        
        # Detectar versión de OpenCV y configurar ArUco
        opencv_version = cv2.__version__.split('.')
        self.opencv_major = int(opencv_version[0])
        self.opencv_minor = int(opencv_version[1])
        
        self.get_logger().info(f"OpenCV versión detectada: {cv2.__version__}")
        
        # Obtener diccionario ArUco
        aruco_dict_id = getattr(cv2.aruco, self.aruco_dict_type)
        self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)
        
        # Crear parámetros del detector según versión de OpenCV
        try:
            # Intentar API nueva (OpenCV 4.7+)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.use_new_api = True
            self.get_logger().info("Usando API nueva de ArUco (OpenCV 4.7+)")
        except AttributeError:
            try:
                # Intentar API intermedia (OpenCV 4.0-4.6)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.use_new_api = False
                self.get_logger().info("Usando API intermedia de ArUco (OpenCV 4.0-4.6)")
            except AttributeError:
                # API muy antigua (OpenCV 3.x)
                self.aruco_params = cv2.aruco.DetectorParameters()
                self.use_new_api = False
                self.get_logger().info("Usando API antigua de ArUco (OpenCV 3.x)")
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publicador de imagen con detecciones
        self.image_pub = self.create_publisher(
            Image, 
            '/aruco/detected_image', 
            10
        )
        
        # Suscriptor a camera_info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Suscriptor a la imagen de la cámara
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ArUco Detector Node iniciado (ROS2)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Cámara imagen: {self.camera_topic}")
        self.get_logger().info(f"Cámara info: {self.camera_info_topic}")
        self.get_logger().info(f"Frame cámara: {self.camera_frame}")
        self.get_logger().info(f"Diccionario ArUco: {self.aruco_dict_type}")
        self.get_logger().info(f"Tamaño marcador: {self.marker_size}m")
        self.get_logger().info("Esperando parámetros de cámara...")
        self.get_logger().info("=" * 60)
    
    def camera_info_callback(self, msg):
        """
        Callback para recibir los parámetros intrínsecos de la cámara desde Gazebo
        """
        if not self.camera_info_received:
            # Extraer matriz de cámara (K)
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            
            # Extraer coeficientes de distorsión (D)
            self.dist_coeffs = np.array(msg.d)
            
            self.camera_info_received = True
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("Parámetros de cámara recibidos desde Gazebo:")
            self.get_logger().info(f"Matriz de cámara (K):\n{self.camera_matrix}")
            self.get_logger().info(f"Coeficientes distorsión (D): {self.dist_coeffs}")
            self.get_logger().info(f"Resolución: {msg.width}x{msg.height}")
            self.get_logger().info("=" * 60)
            
            # Destruir suscripción después de recibir la info
            self.destroy_subscription(self.camera_info_sub)
    
    def image_callback(self, msg):
        # Esperar a tener los parámetros de la cámara
        if not self.camera_info_received:
            self.get_logger().warn("Esperando parámetros de cámara...", throttle_duration_sec=5.0)
            return
        
        try:
            # Convertir mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return
        
        # Convertir a escala de grises
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detectar marcadores ArUco (compatible con todas las versiones)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, 
            self.aruco_dict, 
            parameters=self.aruco_params
        )
        
        # Si se detectaron marcadores
        if ids is not None and len(ids) > 0:
            # Estimar pose de cada marcador
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )
            
            # Procesar cada marcador detectado
            for i, marker_id in enumerate(ids.flatten()):
                # Publicar TF para este marcador
                self.publish_tf(marker_id, rvecs[i], tvecs[i], msg.header.stamp)
                
                # Dibujar marcador y ejes (para visualización)
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                # Dibujar ejes del marcador (compatible con todas las versiones)
                try:
                    # Intentar API nueva
                    cv2.drawFrameAxes(
                        cv_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[i],
                        tvecs[i],
                        self.marker_size * 0.5
                    )
                except AttributeError:
                    # API antigua usa aruco.drawAxis
                    cv2.aruco.drawAxis(
                        cv_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[i],
                        tvecs[i],
                        self.marker_size * 0.5
                    )
                
                # Añadir texto con ID y posición
                corner = corners[i][0][0]
                pos_text = f"ID:{marker_id} ({tvecs[i][0][0]:.2f}, {tvecs[i][0][1]:.2f}, {tvecs[i][0][2]:.2f})"
                cv2.putText(
                    cv_image,
                    pos_text,
                    (int(corner[0]), int(corner[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 255, 0),
                    1
                )
            
            self.get_logger().info(
                f"Detectados {len(ids)} marcadores: {ids.flatten().tolist()}", 
                throttle_duration_sec=2.0
            )
        else:
            # Mostrar mensaje cuando no hay detecciones
            cv2.putText(
                cv_image,
                "No se detectaron marcadores ArUco",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )
        
        # Publicar imagen con detecciones
        try:
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            image_msg.header = msg.header
            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error al publicar imagen: {e}")
    
    def publish_tf(self, marker_id, rvec, tvec, timestamp):
        """
        Publica la transformación TF desde la cámara al marcador ArUco
        """
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Header
        t.header.stamp = timestamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = f"aruco_{marker_id}"
        
        # Traslación
        t.transform.translation.x = float(tvec[0][0])
        t.transform.translation.y = float(tvec[0][1])
        t.transform.translation.z = float(tvec[0][2])
        
        # Convertir vector de rotación a matriz de rotación
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Convertir matriz de rotación a cuaternión usando scipy
        rot = R.from_matrix(rotation_matrix)
        quaternion = rot.as_quat()  # [x, y, z, w]
        
        # Asignar cuaternión
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        
        # Publicar transformación
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
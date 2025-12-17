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
        
        # Declarar parametros
        self.declare_parameter('camera_topic', '/overhead_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/overhead_camera/camera_info')
        self.declare_parameter('camera_frame', 'overhead_camera_link')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.1)
        
        # Obtener parametros
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.aruco_dict_type = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        
        # Variables para parametros de camara
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # Diccionario para almacenar las ultimas posiciones detectadas
        self.detected_markers = {}
        
        # Inicializar CvBridge
        self.bridge = CvBridge()
        
        # Detectar version de OpenCV y configurar ArUco
        opencv_version = cv2.__version__.split('.')
        self.opencv_major = int(opencv_version[0])
        self.opencv_minor = int(opencv_version[1])
        
        self.get_logger().info(f"OpenCV version detectada: {cv2.__version__}")
        
        # Obtener diccionario ArUco
        aruco_dict_id = getattr(cv2.aruco, self.aruco_dict_type)
        self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)
        
        # Crear parametros del detector segun version de OpenCV
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
        
        # Suscriptor a la imagen de la camara
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Timer para imprimir posiciones cada 5 segundos
        self.print_timer = self.create_timer(5.0, self.print_positions_callback)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ArUco Detector Node iniciado (ROS2)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Camara imagen: {self.camera_topic}")
        self.get_logger().info(f"Camara info: {self.camera_info_topic}")
        self.get_logger().info(f"Frame camara: {self.camera_frame}")
        self.get_logger().info(f"Diccionario ArUco: {self.aruco_dict_type}")
        self.get_logger().info(f"Tamaño marcador: {self.marker_size}m")
        self.get_logger().info("Esperando parametros de camara...")
        self.get_logger().info("=" * 60)
    
    def camera_info_callback(self, msg):
        """
        Callback para recibir los parametros intrinsecos de la camara desde Gazebo
        """
        if not self.camera_info_received:
            # Extraer matriz de camara (K)
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            
            # Extraer coeficientes de distorsion (D)
            self.dist_coeffs = np.array(msg.d)
            
            self.camera_info_received = True
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("Parametros de camara recibidos desde Gazebo:")
            self.get_logger().info(f"Matriz de camara (K):\n{self.camera_matrix}")
            self.get_logger().info(f"Coeficientes distorsion (D): {self.dist_coeffs}")
            self.get_logger().info(f"Resolucion: {msg.width}x{msg.height}")
            self.get_logger().info("=" * 60)
            
            # Destruir suscripcion despues de recibir la info
            self.destroy_subscription(self.camera_info_sub)
    
    def print_positions_callback(self):
        """
        Imprime las posiciones de todos los ArUcos detectados cada 5 segundos
        """
        if not self.camera_info_received:
            return
            
        if not self.detected_markers:
            self.get_logger().info("No se han detectado marcadores ArUco aun...")
            return
        
        # Ordenar marcadores por ID
        sorted_markers = sorted(self.detected_markers.items())
        
        self.get_logger().info("")
        self.get_logger().info("=" * 80)
        self.get_logger().info("POSICIONES DE ARUCOS DETECTADOS")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Frame de referencia: {self.camera_frame}")
        self.get_logger().info(f"Total de ArUcos detectados: {len(self.detected_markers)}")
        self.get_logger().info("-" * 80)
        
        for marker_id, (tvec, rvec) in sorted_markers:
            x, y, z = tvec[0]
            
            # Convertir rotacion a angulos de Euler
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            rot = R.from_matrix(rotation_matrix)
            euler = rot.as_euler('xyz', degrees=True)
            
            # Destacar si es el ArUco 2 (origen del sistema)
            if marker_id == 2:
                self.get_logger().info(f"ArUco {marker_id:2d} (ORIGEN): X={x:+7.3f}m  Y={y:+7.3f}m  Z={z:+7.3f}m  | Yaw={euler[2]:+7.2f}°")
            elif marker_id == 3:
                self.get_logger().info(f"ArUco {marker_id:2d} (ROBOT) : X={x:+7.3f}m  Y={y:+7.3f}m  Z={z:+7.3f}m  | Yaw={euler[2]:+7.2f}°")
            else:
                self.get_logger().info(f"ArUco {marker_id:2d}         : X={x:+7.3f}m  Y={y:+7.3f}m  Z={z:+7.3f}m  | Yaw={euler[2]:+7.2f}°")
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("")
    
    def image_callback(self, msg):
        # Esperar a tener los parametros de la camara
        if not self.camera_info_received:
            self.get_logger().warn("Esperando parametros de camara...", throttle_duration_sec=5.0)
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
                # Guardar posicion detectada
                self.detected_markers[marker_id] = (tvecs[i], rvecs[i])
                
                # Publicar TF para este marcador
                self.publish_tf(marker_id, rvecs[i], tvecs[i], msg.header.stamp)
                
                # Dibujar marcador y ejes (para visualizacion)
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
                
                # Añadir texto con ID y posicion
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
        Publica la transformacion TF desde la camara al marcador ArUco
        """
        # Crear mensaje de transformacion
        t = TransformStamped()
        
        # Header
        t.header.stamp = timestamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = f"aruco_{marker_id}"
        
        # Traslacion
        t.transform.translation.x = float(tvec[0][0])
        t.transform.translation.y = float(tvec[0][1])
        t.transform.translation.z = float(tvec[0][2])
        
        # Convertir vector de rotacion a matriz de rotacion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Convertir matriz de rotacion a cuaternion usando scipy
        rot = R.from_matrix(rotation_matrix)
        quaternion = rot.as_quat()  # [x, y, z, w]
        
        # Asignar cuaternion
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        
        # Publicar transformacion
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
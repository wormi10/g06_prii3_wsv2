
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import threading
import time
import math

# ============================================================
#  CONFIGURACIÓN GLOBAL (ajusta aquí sin tocar el resto)
# ============================================================

ORDEN_DESEADO       = [20, 21, 22, 23]   # Ruta a seguir
LINEAR_SPEED        = 0.15               # m/s avance recto
ANGULAR_SPEED       = 0.4               # rad/s giro
DISTANCE_TOLERANCE  = 40.0              # píxeles para considerar "llegado"
WAIT_TIME           = 3.0               # segundos de espera en cada punto
START_DELAY         = 3.0               # segundos de espera inicial
STALE_TIMEOUT       = 1.5              # segundos sin dato → ignora lectura

# Corrección continua de rumbo mientras avanza (PD sobre el ángulo)
KP_HEADING          = 0.008            # ganancia proporcional de dirección
KD_HEADING          = 0.002            # ganancia derivativa de dirección
MAX_HEADING_CORRECT = 0.35             # límite de corrección angular (rad/s)


class JetBotNavigation(Node):
    def __init__(self):
        super().__init__('movimiento_JetBot')

        
        self.listen_ids = [3, 8, 20, 21, 22, 23]
        for tid in self.listen_ids:
            topic = f'/overhead_camera/aruco_{tid}'
            self.create_subscription(
                String,
                topic,
                lambda msg, tid=tid: self.aruco_cb(msg, tid),
                10
            )

        # --------------------------------------------------------
        # Publicador de velocidad
        # --------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --------------------------------------------------------
        # Estado compartido (acceso siempre bajo self.lock)
        # --------------------------------------------------------
        self.raw  = {}          # {id: {'px', 'py', 'orientation', 'last_seen'}}
        self.lock = threading.Lock()

        # --------------------------------------------------------
        # Máquina de estados
        # --------------------------------------------------------
        self.state           = 'WAITING'
        self.target_id       = None
        self.visited         = []
        self.start_time      = time.time()

        # Variables para el controlador PD de rumbo
        self._prev_heading_error = 0.0
        self._prev_heading_time  = time.time()

        # Variable auxiliar para el estado WAITING_AT_POINT
        self._wait_start = None

        # --------------------------------------------------------
        # Timer principal de navegación (10 Hz)
        # --------------------------------------------------------
        self.create_timer(0.1, self.navigation_loop)

        self.get_logger().info("=" * 55)
        self.get_logger().info("  JetBot Navigation arrancado")
        self.get_logger().info(f"  Ruta planificada: {ORDEN_DESEADO}")
        self.get_logger().info(f"  Esperando {START_DELAY}s antes de mover...")
        self.get_logger().info("=" * 55)

    # ==========================================================
    #  CALLBACK ArUco
    # ==========================================================

    def aruco_cb(self, msg: String, tid: int):
        """
        Recibe JSON con px, py, orientation para cada ArUco.
        Actualiza self.raw de forma thread-safe.
        """
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        try:
            px          = float(data.get('px', 0.0))
            py          = float(data.get('py', 0.0))
            orientation = float(data.get('orientation', 0.0))
        except Exception:
            return

        with self.lock:
            self.raw[tid] = {
                'px':          px,
                'py':          py,
                'orientation': orientation,
                'last_seen':   time.time()
            }

    # ==========================================================
    #  HELPERS DE LECTURA
    # ==========================================================

    def get_pose(self, tid):
        """
        Devuelve (px, py, orientation) del ArUco `tid`
        o None si no hay dato reciente.
        """
        with self.lock:
            entry = self.raw.get(tid)

        if entry is None:
            return None
        if (time.time() - entry['last_seen']) > STALE_TIMEOUT:
            return None   # dato obsoleto

        return entry['px'], entry['py'], entry['orientation']

    def pixel_distance(self, ax, ay, bx, by):
        """Distancia euclidiana en píxeles entre dos puntos."""
        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

    def pixel_angle(self, from_x, from_y, to_x, to_y):
        """
        Ángulo (rad) del vector que va de (from) a (to)
        en coordenadas de imagen (Y apunta hacia abajo).
        """
        return math.atan2(to_y - from_y, to_x - from_x)

    # ==========================================================
    #  CONTROL DE VELOCIDAD
    # ==========================================================

    def publish_vel(self, lin, ang):
        msg = Twist()
        msg.linear.x  = float(lin)
        msg.angular.z = float(ang)
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.publish_vel(0.0, 0.0)

    # ==========================================================
    #  MÁQUINA DE ESTADOS  (llamada a 10 Hz)
    # ==========================================================

    def navigation_loop(self):

        # --------------------------------------------------
        # 0. ESPERA INICIAL antes de arrancar
        # --------------------------------------------------
        if self.state == 'WAITING':
            if (time.time() - self.start_time) > START_DELAY:
                self.get_logger().info("Arrancando navegación...")
                self.state = 'IDLE'
            return

        # --------------------------------------------------
        # 1. IDLE → seleccionar siguiente objetivo
        # --------------------------------------------------
        if self.state == 'IDLE':
            unvisited = [a for a in ORDEN_DESEADO if a not in self.visited]

            if not unvisited:
                self.get_logger().info("Ruta completa. Reiniciando ciclo...")
                self.visited = []
                unvisited = list(ORDEN_DESEADO)

            self.target_id = unvisited[0]
            self.visited.append(self.target_id)
            self._prev_heading_error = 0.0
            self._prev_heading_time  = time.time()
            self.get_logger().info(f"─── NUEVO DESTINO → ArUco {self.target_id} ───")
            self.state = 'TURNING'
            return

        # --------------------------------------------------
        # 2. TURNING → gira hasta apuntar al objetivo
        # --------------------------------------------------
        if self.state == 'TURNING':
            robot  = self.get_pose(3)
            target = self.get_pose(self.target_id)

            if robot is None or target is None:
                self.get_logger().warn("⚠ Sin datos de cámara, esperando...")
                self.stop()
                return

            rx, ry, robot_orientation = robot
            tx, ty, _                 = target

            # Ángulo hacia el objetivo en la imagen
            angle_to_target = self.pixel_angle(rx, ry, tx, ty)

            # Error de heading (diferencia normalizada entre -π y π)
            heading_error = self._normalize_angle(angle_to_target - robot_orientation)

            self.get_logger().info(
                f"  TURNING | error={math.degrees(heading_error):.1f}°",
                throttle_duration_sec=0.5
            )

            # Si el error es pequeño, ya estamos alineados
            if abs(heading_error) < math.radians(8):
                self.stop()
                time.sleep(0.3)
                self.state = 'GOING'
                self.get_logger().info("  ✓ Alineado. Avanzando...")
                return

            # Girar en la dirección correcta
            direction = 1.0 if heading_error > 0 else -1.0
            self.publish_vel(0.0, ANGULAR_SPEED * direction)
            return

        # --------------------------------------------------
        # 3. GOING → avanza recto con corrección de rumbo
        # --------------------------------------------------
        if self.state == 'GOING':
            robot  = self.get_pose(3)
            target = self.get_pose(self.target_id)

            if robot is None or target is None:
                self.get_logger().warn("⚠ Sin datos de cámara, parando.")
                self.stop()
                return

            rx, ry, robot_orientation = robot
            tx, ty, _                 = target

            dist = self.pixel_distance(rx, ry, tx, ty)

            # ¿Llegamos?
            if dist <= DISTANCE_TOLERANCE:
                self.stop()
                self.get_logger().info(f"  ✓ Llegado al ArUco {self.target_id}!")
                self._wait_start = time.time()
                self.state = 'WAITING_AT_POINT'
                return

            # ── Corrección continua de rumbo (controlador PD) ──
            angle_to_target = self.pixel_angle(rx, ry, tx, ty)
            heading_error   = self._normalize_angle(angle_to_target - robot_orientation)

            now    = time.time()
            dt     = max(now - self._prev_heading_time, 1e-3)
            d_term = (heading_error - self._prev_heading_error) / dt

            angular_correction = (KP_HEADING * heading_error) + (KD_HEADING * d_term)
            angular_correction = max(-MAX_HEADING_CORRECT,
                                     min(MAX_HEADING_CORRECT, angular_correction))

            self._prev_heading_error = heading_error
            self._prev_heading_time  = now

            self.get_logger().info(
                f"  GOING | dist={dist:.0f}px | err={math.degrees(heading_error):.1f}° | corr={angular_correction:.3f}",
                throttle_duration_sec=0.5
            )

            self.publish_vel(LINEAR_SPEED, angular_correction)
            return

        # --------------------------------------------------
        # 4. WAITING_AT_POINT → espera antes del siguiente
        # --------------------------------------------------
        if self.state == 'WAITING_AT_POINT':
            self.stop()
            elapsed = time.time() - self._wait_start

            if elapsed >= WAIT_TIME:
                self.get_logger().info(
                    f"  Espera completada. Siguiente destino..."
                )
                self.state = 'IDLE'
            return

    # ==========================================================
    #  UTILIDADES
    # ==========================================================

    def _normalize_angle(self, angle):
        """Normaliza un ángulo al rango [-π, π]."""
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle


# ==========================================================
#  MAIN
# ==========================================================

def main(args=None):
    rclpy.init(args=args)
    node = JetBotNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
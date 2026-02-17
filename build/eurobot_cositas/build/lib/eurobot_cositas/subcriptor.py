import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
import threading
import json
import time
from typing import Optional, Dict, Any


class RobotListener(Node):
    """Nodo que escucha tópicos típicos y los marcadores ArUco.
    Mantiene el último mensaje recibido por tópico y cada segundo imprime un
    resumen en consola.
    """
    def __init__(self):
        super().__init__('robot_listener')

        # tópicos ArUco (por ID)
        self.listen_ids = [3, 8, 20, 21, 22, 23]
        for tid in self.listen_ids:
            topic = f'/overhead_camera/aruco_{tid}'
            self.create_subscription(String, topic, lambda msg, tid=tid: self.aruco_cb(msg, tid), 10)

        # tópicos "típicos" que se desean observar
        typical = [
            ('/cmd_vel', Twist),
            ('/odom', Odometry),
            ('/scan', LaserScan),
            ('/imu', Imu),
            ('/chatter', String),
        ]
        for topic, mtype in typical:
            try:
                self.create_subscription(mtype, topic, lambda msg, t=topic: self.generic_cb(msg, t), 10)
            except Exception:
                # si el topic/mtype no está disponible en el entorno, seguimos
                pass

        # almacenamiento de últimos mensajes y sincronización
        self.last_msgs: Dict[str, Dict[str, Any]] = {}
        # raw mantiene un dict por id de ArUco para compatibilidad con vuestro código
        # raw[id] = {'px','py','pz','orientation','last_seen'}
        self.raw: Dict[int, Dict[str, Any]] = {}
        self.lock = threading.Lock()

        # timer para imprimir cada segundo
        self.create_timer(1.0, self._print_status)

    def aruco_cb(self, msg: String, tid: int):
        """Callback para tópicos ArUco: espera JSON en msg.data."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        try:
            px = float(data.get('px', 0.0))
            py = float(data.get('py', 0.0))
            pz = float(data.get('pz', 0.0))
            orientation = float(data.get('orientation', 0.0))
        except Exception:
            return

        with self.lock:
            self.last_msgs[f'/overhead_camera/aruco_{tid}'] = {
                'type': 'aruco',
                'data': {'px': px, 'py': py, 'pz': pz, 'orientation': orientation},
                'time': time.time()
            }
            # mantener también en self.raw por id (compatibilidad con snippet dado)
            # ahora incluimos 'pz' para que get_aruco/proveedores externos lo reciban
            self.raw[tid] = {'px': px, 'py': py, 'pz': pz, 'orientation': orientation, 'last_seen': time.time()} 

    def generic_cb(self, msg: Any, topic: str):
        """Callback genérico para guardar el último mensaje recibido por tópico."""
        with self.lock:
            self.last_msgs[topic] = {'type': type(msg).__name__, 'data': msg, 'time': time.time()}

    def _summarize(self, entry: Dict[str, Any]) -> str:
        t = entry['type']
        data = entry['data']
        if t == 'Twist' or t == 'TwistStamped':
            # geometry_msgs/Twist
            try:
                return f"Twist lin.x={data.linear.x:.2f} ang.z={data.angular.z:.2f}"
            except Exception:
                return str(data)
        if t == 'Odometry':
            try:
                px = data.pose.pose.position.x
                py = data.pose.pose.position.y
                return f"Odom x={px:.2f} y={py:.2f}"
            except Exception:
                return str(data)
        if t == 'LaserScan':
            try:
                n = len(data.ranges)
                mn = min(data.ranges) if n > 0 else float('nan')
                return f"LaserScan ranges={n} min={mn:.2f}"
            except Exception:
                return str(data)
        if t == 'Imu':
            try:
                ax = data.linear_acceleration.x
                ay = data.linear_acceleration.y
                az = data.linear_acceleration.z
                return f"Imu a=({ax:.2f},{ay:.2f},{az:.2f})"
            except Exception:
                return str(data)
        if t == 'String':
            try:
                return f"String: {data.data}"
            except Exception:
                return str(data)
        if t == 'aruco':
            d = data
            # mostrar también la componente z y mantener formato compacto
            return f"Aruco px={d['px']:.2f} py={d['py']:.2f} pz={d.get('pz', 0.0):.2f} ori={d['orientation']:.2f}"
        # fallback
        return str(data)

    def _print_status(self):
        now = time.time()
        lines = []
        with self.lock:
            if not self.last_msgs:
                lines.append('No messages received yet')
            else:
                # cabecera tipo tabla para lectura más cómoda
                lines.append(f"{'TOPIC':<36} {'TYPE':<10} {'AGE':>6}  SUMMARY")
                lines.append('-' * 88)
                for topic, entry in sorted(self.last_msgs.items()):
                    age = now - entry['time']
                    summary = self._summarize(entry)
                    lines.append(f"{topic:<36} {entry['type']:<10} {age:5.1f}s  {summary}")

        # imprimir fuera del lock
        for l in lines:
            self.get_logger().info(l)

    def get_aruco(self, tid: int) -> Optional[Dict[str, float]]:
        """Devuelve la última posición conocida del aruco con ID tid.
        Si no se ha visto en los últimos 0.5s, devuelve None.
        Mantiene la misma interfaz que su snippet original.
        """
        with self.lock:
            info = self.raw.get(tid, None)
            if info is None:
                return None
            if time.time() - info['last_seen'] > 0.5:
                return None
            return info


def main(args=None):
    rclpy.init(args=args)
    node = RobotListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

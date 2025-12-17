Iiniciar Tablero Eurobot Cositas
Los modelos en la carpeta de models colocalos en tu carpeta .gazebo/models
```bash
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle

colcon build
source install/setup.bash
# Solo detector Aruco
ros2 launch eurobot_cositas eurobot_world_launch.py
# Detección y movimiento.
ros2 launch eurobot_cositas Move_eurobot_launch.py
```
PBI 4.2 – Localización de marcadores ArUco con cámara cenital

Se ha implementado un sistema de localización basado en visión utilizando una cámara cenital.
El workspace incluye:

Un nodo ROS que detecta y localiza los marcadores ArUco del tablero Eurobot2026.

Un nodo ROS específico para la localización del marcador ArUco del robot.

Publicación de las transformaciones TF desde la cámara cenital hacia cada marcador detectado.

Visualización correcta de los marcos de referencia de los ArUco y del robot en RViz, verificando la coherencia espacial del sistema.

La funcionalidad completa se ejecuta desde un único fichero launch.

<img width="1858" height="702" alt="image" src="https://github.com/user-attachments/assets/cca539e0-91f8-41f5-a54b-82fc3f63bda7" />

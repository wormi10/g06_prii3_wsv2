Iiniciar Tablero Eurobot Cositas
Los modelos en la carpeta de models colocalos en tu carpeta .gazebo/models

source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle

colcon build
source install/setup.bash
ros2 launch eurobot_cositas Move_eurobot_launch.py


colcon build 
source install/setup.bash
ros2 launch eurobot_cositas eurobot_world_launch.py 

otra terminal
ros2 run rqt_image_view rqt_image_view /template/detected_image

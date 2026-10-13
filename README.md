# Simulación y Control de Robot Diferencial en ROS 2

Simulación y control cinemático de un robot móvil de tracción diferencial (dos ruedas motrices y casters de apoyo) en un circuito de obstáculos. Este proyecto implementa la cinemática directa calculada analíticamente y validada en tiempo real contra los datos de odometría del simulador.

## Tecnologías y Herramientas
* **Framework:** ROS 2 (Jazzy)
* **Simulador:** Gazebo Harmonic & RViz2
* **Lenguajes:** Python (Nodo de cinemática), XML/Xacro (Modelo URDF), MATLAB (Cálculo simbólico de la matriz Jacobiana)

## Especificaciones del Vehículo y Entorno
* **Odometría en Tiempo Real:** Nodo personalizado en Python (`cinematica_llantas.py`) que lee la velocidad de las ruedas en `/joint_states`, aplica integración de Euler para calcular la pose y reporta el margen de error frente a `/odom`.
* **Sensores Justificados:** 
  * **LiDAR (Slamtec RPLiDAR A1):** Rango 0.12m a 8.0m, escaneo 360° a 10Hz.
  * **Cámara (Logitech C270):** Resolución VGA (640x480), 30 FPS, FOV de 60°.
* **Pista de Pruebas:** Circuito cerrado con físicas de colisión activas, incluyendo una chicana y una línea de 4 obstáculos para pruebas de slalom.

## Instalación y Ejecución

1. Clonar el repositorio en la carpeta `src` de tu workspace:
   ```bash
   cd ~/ros2_ws_dif_C2/src
   git clone [https://github.com/IsaGuerrero33/Diff_robot_wheels.git](https://github.com/IsaGuerrero33/Diff_robot_wheels.git)

2. Compilar los paquetes del modelo y de control:
    Bash

    cd ~/ros2_ws_dif_C2
    colcon build
    source install/setup.bash

3. Lanzar la simulación (Gazebo Harmonic con la pista de obstáculos):
    Bash

    ros2 launch mi_robot_diferencial simulacion.launch.py

4. En una nueva terminal, ejecutar el nodo de cálculo cinemático para ver el error en consola:
    Bash

    cd ~/ros2_ws_dif_C2
    source install/setup.bash
    ros2 run mi_robot_control cinematica_llantas

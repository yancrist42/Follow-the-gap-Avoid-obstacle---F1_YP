# README — Nodo `ReactiveFollowGap` (`obstacle_node.py`) y Rebase en Pista

Documentación complementaria para el repositorio (junto al `README.md` base de `f1tenth_gym_ros`) sobre el nodo `ReactiveFollowGap` (`obstacle_node.py`), ubicado en la carpeta `controller` junto a `gap_node`.

---

## Descripción del enfoque utilizado

El nodo `ReactiveFollowGap` implementa un controlador reactivo basado en **Follow the Gap**, moderación de velocidad (*Trail Braking*) y telemetría de vueltas sin requerir un mapa previo de la pista:

* **Preprocesamiento del LIDAR (`preprocess_lidar`):** Convierte las lecturas de `/scan` (`LaserScan`) en un arreglo de NumPy, recorta la distancia máxima (por ejemplo `3.9` m en la configuración base estable) y fija en `0.0` los rayos fuera del sector frontal de `-70°` a `+70°`.
* **Burbuja de seguridad (`lidar_callback`):** Identifica el punto válido más cercano (`closest_dist`) dentro del sector frontal, calcula el ángulo equivalente a `self.bubble_radius` (`0.62` m en la base) y borra (`0.0`) los índices cercanos para evitar acercarse demasiado a muros u obstáculos.
* **Búsqueda de brecha y punto objetivo (`find_max_gap` y `find_best_point`):** Encuentra la secuencia continua más larga de lecturas mayores a `0.01` y selecciona un punto objetivo dentro de esa brecha (por ejemplo el índice central entre los valores máximos).
* **Dirección y control de velocidad (`lidar_callback`):** Convierte el índice seleccionado a `steering_angle` limitado a `[-0.4, 0.4]` y calcula `speed` escalada a `max_speed` (`10.0` por defecto) según el giro, la distancia frontal (`front_dist`) y la cercanía a obstáculos (`closest_dist`).
* **Telemetría de vueltas (`odom_callback`):** Lee `(x, y)` desde `/ego_racecar/odom`, fija la línea de meta en la posición inicial, arma el sensor al superar `10.0` m de distancia al inicio y registra cada vuelta y su duración al volver a menos de `4.5` m.

---

## Estructura del código

En la carpeta `controller` conviven `gap_node` y el nodo de evasión/rebase (`obstacle_node.py`). Dentro de `obstacle_node.py` el código se divide en:

* **`ReactiveFollowGap.__init__`:** Configura el nodo `'reactive_node'`, las suscripciones a `/scan` y `/ego_racecar/odom`, el publicador a `/drive` (`AckermannDriveStamped`), `self.bubble_radius`, los parámetros `max_speed` y `car_label`, y las variables de telemetría (`start_x`, `start_y`, `lap_start_time`, `lap_count`, `is_away`).
* **`odom_callback`:** Gestiona la línea de meta, el estado `is_away` (> `10.0` m) y el conteo/cronómetro de vueltas (< `4.5` m).
* **`preprocess_lidar`:** Aplica el recorte de distancia y la ventana angular de `-70°` a `+70°`.
* **`find_max_gap`:** Localiza los índices de inicio y fin de la brecha continua más amplia.
* **`find_best_point`:** Elige el índice objetivo dentro de la brecha seleccionada.
* **`lidar_callback`:** Ejecuta en cada escaneo la detección del obstáculo más cercano, la burbuja, la búsqueda de brecha, el cálculo de `steering_angle` y `speed`, y la publicación en `/drive`.
* **`main`:** Inicia `rclpy`, ejecuta `rclpy.spin`, captura `KeyboardInterrupt` (`Ctrl+C`) para cerrar la telemetría limpia y destruye el nodo y `rclpy` en `finally`.

---

## Instrucciones de ejecución

Asumiendo que ROS 2, `f1tenth_gym`, `f1tenth_gym_ros` y el entorno de simulación ya están instalados y compilados según el `README.md` principal, sigue estos pasos para ejecutar este nodo en escenarios con obstáculos o con dos agentes en pista.

### 1. Modificaciones en `sim.yaml` y mapas
Edita `f1tenth_gym_ros/config/sim.yaml` según el escenario que quieras probar:

* **Cambiar de mapa (`map_path`):** Apunta `map_path` al mapa deseado, ya sea el mapa normal o el mapa con obstáculos **`Catalunya_Obstacle`** (usando la ruta correspondiente en el entorno o contenedor).
* **Configurar uno o dos agentes (`num_agent`):** Cambia `num_agent` a `1` (un solo agente, por ejemplo sobre `Catalunya_Obstacle`) o a `2` (dos agentes para pruebas de rebase).
* **Verificar poses y tópicos:** Revisa las poses iniciales (`ego` / `opponent`) si lo requieres. Con dos agentes, el simulador publica `/scan`, `/ego_racecar/odom`, `/opp_scan` y `/opp_racecar/odom`, y suscribe a `/drive` y `/opp_drive` (es necesario publicar en ambos tópicos de conducción para que ambos vehículos se muevan).

[Screencast from 11-22-2026 12:02:12 PM.webm](https://github.com/user-attachments/assets/a924139d-6b75-4790-9941-ca0188ce2825)

### 2. Modificaciones en el nodo y compilación
* Mantén o actualiza `obstacle_node.py` dentro de la carpeta `controller` (`~/f1tenth_ws/src/controllers/controllers/obstacle_node.py`), ajustando parámetros como `max_speed`, `self.bubble_radius` o el recorte de distancia en `preprocess_lidar` si deseas afinar la conducción o el rebase.
* Si modificas `sim.yaml` o paquetes de ROS 2, recompila y vuelve a cargar el entorno:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. Lanzar la simulación y los nodos

#### Paso 3.1 — Iniciar la simulación

En una terminal, arranca el entorno de simulación:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
> **Nota visual en RViz (modo 2 agentes):** Aunque configures `num_agent: 2` en `sim.yaml`, RViz por defecto solo dibuja el modelo 3D del vehículo principal (`ego_racecar`), por lo que el segundo vehículo (`opp_racecar`) aparecerá inicialmente solo como el contorno de puntos de su carrocería en el `LaserScan`. Para visualizar también el modelo 3D del oponente en RViz:
> 1. En el panel izquierdo (**Displays**), haz clic en **Add** y selecciona **RobotModel**.
> 2. Despliega el nuevo **RobotModel** agregado y cambia su propiedad **Description Topic** a `/opp_robot_description` (o `/opp_racecar/robot_description`).
> 3. *(Opcional)* Guarda la configuración desde **File -> Save Config**.

#### Paso 3.2 — Ejecutar el agente principal (`EGO`)

En otra terminal, lanza el nodo para el vehículo `EGO`:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash

python3 ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/src/controllers/controllers/obstacle_node.py --ros-args \
  -r __node:=ego_gap_node \
  -p max_speed:=16.0 \
  -p car_label:=EGO
```

#### Paso 3.3 — Ejecutar el segundo agente (`OPP`) *(solo con `num_agent: 2`)*

En una tercera terminal, lanza el nodo para el oponente `OPP` remapeando sus tópicos de LIDAR, conducción y odometría:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash

python3 ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/src/controllers/controllers/obstacle_node.py --ros-args \
  -r __node:=opp_gap_node \
  -r /scan:=/opp_scan \
  -r /drive:=/opp_drive \
  -r /ego_racecar/odom:=/opp_racecar/odom \
  -p max_speed:=4.0 \
  -p car_label:=OPP
```

Para detener cualquiera de los nodos de forma limpia y mostrar el mensaje de cierre de telemetría, presiona `Ctrl + C` en su terminal.

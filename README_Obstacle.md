# README — Nodo `ReactiveFollowGap` (`obstacle_node.py`) y Rebase en Pista

Documentación complementaria para el repositorio (junto al `README.md` base de `f1tenth_gym_ros`) sobre el nodo `ReactiveFollowGap` (`obstacle_node.py`), ubicado en la carpeta `controller` junto a `gap_node`.

---

## Descripción del enfoque utilizado

El nodo `ReactiveFollowGap` implementa un controlador reactivo basado en **Follow the Gap**, moderación de velocidad (*Trail Braking*) y telemetría de vueltas sin requerir un mapa previo de la pista[cite: 1]:

* **Preprocesamiento del LIDAR (`preprocess_lidar`):** Convierte las lecturas de `/scan` (`LaserScan`) en un arreglo de NumPy, recorta la distancia máxima (por ejemplo `3.9` m en la configuración base estable) y fija en `0.0` los rayos fuera del sector frontal de `-70°` a `+70°`[cite: 1].
* **Burbuja de seguridad (`lidar_callback`):** Identifica el punto válido más cercano (`closest_dist`) dentro del sector frontal, calcula el ángulo equivalente a `self.bubble_radius` (`0.62` m en la base) y borra (`0.0`) los índices cercanos para evitar acercarse demasiado a muros u obstáculos[cite: 1].
* **Búsqueda de brecha y punto objetivo (`find_max_gap` y `find_best_point`):** Encuentra la secuencia continua más larga de lecturas mayores a `0.01` y selecciona un punto objetivo dentro de esa brecha (por ejemplo el índice central entre los valores máximos)[cite: 1].
* **Dirección y control de velocidad (`lidar_callback`):** Convierte el índice seleccionado a `steering_angle` limitado a `[-0.4, 0.4]` y calcula `speed` escalada a `max_speed` (`10.0` por defecto) según el giro, la distancia frontal (`front_dist`) y la cercanía a obstáculos (`closest_dist`)[cite: 1].
* **Telemetría de vueltas (`odom_callback`):** Lee `(x, y)` desde `/ego_racecar/odom`, fija la línea de meta en la posición inicial, arma el sensor al superar `10.0` m de distancia al inicio y registra cada vuelta y su duración al volver a menos de `4.5` m[cite: 1].

---

## Estructura del código

En la carpeta `controller` conviven `gap_node` y el nodo de evasión/rebase (`obstacle_node.py`)[cite: 1]. Dentro de `obstacle_node.py` el código se divide en:

* **`ReactiveFollowGap.__init__`:** Configura el nodo `'reactive_node'`, las suscripciones a `/scan` y `/ego_racecar/odom`, el publicador a `/drive` (`AckermannDriveStamped`), `self.bubble_radius`, los parámetros `max_speed` y `car_label`, y las variables de telemetría (`start_x`, `start_y`, `lap_start_time`, `lap_count`, `is_away`)[cite: 1].
* **`odom_callback`:** Gestiona la línea de meta, el estado `is_away` (> `10.0` m) y el conteo/cronómetro de vueltas (< `4.5` m)[cite: 1].
* **`preprocess_lidar`:** Aplica el recorte de distancia y la ventana angular de `-70°` a `+70°`[cite: 1].
* **`find_max_gap`:** Localiza los índices de inicio y fin de la brecha continua más amplia[cite: 1].
* **`find_best_point`:** Elige el índice objetivo dentro de la brecha seleccionada[cite: 1].
* **`lidar_callback`:** Ejecuta en cada escaneo la detección del obstáculo más cercano, la burbuja, la búsqueda de brecha, el cálculo de `steering_angle` y `speed`, y la publicación en `/drive`[cite: 1].
* **`main`:** Inicia `rclpy`, ejecuta `rclpy.spin`, captura `KeyboardInterrupt` (`Ctrl+C`) para cerrar la telemetría limpia y destruye el nodo y `rclpy` en `finally`[cite: 1].

---

## Instrucciones de ejecución

Asumiendo que ROS 2, `f1tenth_gym`, `f1tenth_gym_ros` y el entorno de simulación ya están instalados y compilados según el `README.md` principal[cite: 2], sigue estos pasos para ejecutar este nodo en escenarios de obstáculos o dos agentes:

### 1. Modificaciones en `sim.yaml` y mapas
Edita `f1tenth_gym_ros/config/sim.yaml` según el escenario que quieras probar[cite: 2]:

* **Cambiar de mapa (`map_path`):** Apunta `map_path` al mapa correspondiente en el repositorio/contenedor[cite: 2], ya sea el mapa normal o el mapa con obstáculos **`Catalunya_Obstacle`** (siguiendo la convención de ruta completa del mapa en el contenedor)[cite: 2].
* **Configurar uno o dos agentes (`num_agent`):** Ajusta `num_agent` a `1` (un agente, por ejemplo sobre el mapa `Catalunya_Obstacle`) o a `2` (dos agentes para pruebas de rebase en pista)[cite: 2].
* **Revisar posiciones iniciales y tópicos:** Verifica las poses iniciales (`ego` / `opponent`) en `sim.yaml` si es necesario[cite: 2]. Ten en cuenta que con `num_agent: 2` el simulador publica `/scan`, `/ego_racecar/odom`, `/opp_scan` y `/opp_racecar/odom`, y suscribe a `/drive` y `/opp_drive` (si usas un segundo agente, también requiere publicar en `/opp_drive` para que ambos vehículos avancen)[cite: 2].

### 2. Modificaciones en el nodo y recompilación
* Coloca o actualiza `obstacle_node.py` dentro de la carpeta `controller` de tu paquete en el espacio de trabajo, y ajusta parámetros como `max_speed`, `self.bubble_radius` o el límite de distancia en `preprocess_lidar` si deseas afinar el rebase[cite: 1].
* Recompila y vuelve a cargar el entorno del espacio de trabajo tras modificar `sim.yaml` o el código[cite: 2]:

```bash
cd /sim_ws
colcon build
source install/setup.bash

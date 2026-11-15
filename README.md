# F1Tenth Autonomous Racing: Controlador Reactivo "Follow the Gap" con Telemetría y Trail Braking

Este repositorio contiene una implementación avanzada del algoritmo clásico **Follow-the-Gap (FtG)** optimizada para pruebas de resistencia (*Endurance*) en el simulador **F1Tenth**, utilizando **ROS 2 Humble**. 

El sistema combina navegación reactiva basada en LiDAR con un perfil de velocidad tipo *Trail Braking* y un módulo integrado de telemetría por odometría para el registro automático de tiempos por vuelta. Además, incluye mapas personalizados (Circuito de Catalunya) con obstáculos estáticos para poner a prueba la evasión de colisiones y el tiempo de reacción.

*(Arrastra y suelta aquí un GIF o video corto de tu simulación funcionando)*

---

## 1. Arquitectura de Control y Enfoque Teórico

Para lograr un equilibrio entre velocidad máxima en rectas y estabilidad absoluta en curvas cerradas (completando múltiples vueltas consecutivas sin colisiones a un promedio de ~42 segundos por vuelta), el controlador implementa los siguientes módulos:

1. **Preprocesamiento y Campo de Visión (FoV Filtering):**
   * Se recorta la nube de puntos del LiDAR en un rango frontal estricto de **-70° a +70°** para evitar el "secuestro de burbuja" causado por las paredes laterales cercanas.
   * Se aplica un límite superior de distancia (`np.clip` a **3.9 metros**) para evitar que el algoritmo intente anticipar salidas lejanas y corte los vértices internos de forma prematura (*corner-cutting*).

2. **Evasión de Obstáculos por Burbuja de Seguridad:**
   * Se identifica el punto más cercano al vehículo y se "infla" una burbuja de seguridad con un radio dinámico de **`0.60 metros`**, cegando temporalmente esos puntos para forzar al coche a buscar la brecha (*gap*) más amplia de espacio libre.

3. **Control de Velocidad por Trail Braking Escalonado:**
   * En lugar de frenar de golpe (lo que genera sobreviraje o trompos), el sistema utiliza la lectura promediada de los rayos láser frontales para aplicar una desaceleración gradual (escalones de 14.0 m/s $\rightarrow$ 9.5 m/s $\rightarrow$ 7.5 m/s $\rightarrow$ 6.0 m/s) al acercarse a una curva u obstáculo.
   * La velocidad en curva está acoplada directamente al ángulo de dirección (`steering_angle`), garantizando tracción óptima en arcos amplios y cerrados.

4. **Sistema de Telemetría por Odometría (`/ego_racecar/odom`):**
   * Un submódulo independiente captura la posición bidimensional del vehículo al arrancar, registra ese punto como la línea de meta y calcula con precisión el tiempo transcurrido por vuelta, imprimiendo resultados limpios en la consola de la terminal.

---

## 2. Estructura del Código

El núcleo funcional del controlador reside en el archivo `gap_node.py`. Este script define la clase `ReactiveFollowGap`, un nodo de ROS 2 que interactúa con el motor de físicas mediante:

* **Suscriptores:**
  * `/scan` (`sensor_msgs/msg/LaserScan`): Recibe las lecturas del entorno y alimenta la función `lidar_callback`.
  * `/ego_racecar/odom` (`nav_msgs/msg/Odometry`): Rastrea la posición espacial a través de la función `odom_callback` para el cronometraje de vueltas.
* **Publicador:**
  * `/drive` (`ackermann_msgs/msg/AckermannDriveStamped`): Envía los comandos calculados de aceleración y ángulo de dirección a los actuadores del simulador.
* **Funciones Clave:**
  * `preprocess_lidar`: Filtra los ángulos muertos y recorta el rango efectivo.
  * `find_max_gap` & `find_best_point`: Analizan la matriz de espacio libre para determinar la trayectoria.

---

## 3. Guía de Instalación y Uso (Desde Cero)

Esta guía detalla los pasos verificados para instalar el simulador base de F1Tenth desde cero en un entorno limpio, integrar este controlador reactivo y ejecutar la simulación en el mapa personalizado de Catalunya.

### 3.1. Requisitos Previos
* **Sistema Operativo:** Ubuntu 22.04 LTS
* **Middleware:** ROS 2 Humble instalado (`source /opt/ros/humble/setup.bash`).
* **Herramientas base:** Ejecuta el siguiente bloque para asegurar que tienes `pip`, `colcon` y `rosdep` listos:
  ```bash
  sudo apt update
  sudo apt install -y python3-pip python3-colcon-common-extensions python3-rosdep git
  sudo rosdep init 2>/dev/null || true
  rosdep update
  ```

### 3.2. Instalar el Motor de Físicas (`f1tenth_gym`) y Crear el Workspace
> **Nota importante:** Se debe utilizar la rama `v0.2` (compatible con ROS 2 Humble) y mantener `numpy<2.0` para evitar conflictos de compatibilidad con `numba` y `transforms3d`.

```bash
# 1. Clonar e instalar la versión compatible del motor de físicas
cd ~
git clone -b v0.2 https://github.com/f1tenth/f1tenth_gym.git
cd f1tenth_gym
pip3 install "numpy<2.0" transforms3d
pip3 install -e gym/

# 2. Crear el espacio de trabajo de ROS 2
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# 3. Clonar el puente del simulador oficial para ROS 2
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
```

### 3.3. Clonar este Controlador Reactivo
En la misma carpeta `src` de tu workspace, clona este repositorio para agregar el algoritmo *Follow the Gap* y los mapas personalizados:
```bash
cd ~/f1tenth_ws/src
git clone https://github.com/yancrist42/Follow-the-gap-Avoid-obstacle---F1_YP.git
```

### 3.4. Configurar el Mapa Personalizado
Traslada los archivos del circuito de Catalunya al paquete del simulador y actualiza automáticamente la ruta en `sim.yaml`:

```bash
# 1. Copiar los archivos del mapa (.png y .yaml)
cp ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/maps/Catalunya* ~/f1tenth_ws/src/f1tenth_gym_ros/maps/

# 2. Configurar automáticamente map_path y la extensión .png en sim.yaml
sed -i "s|map_path:.*|map_path: '/home/$USER/f1tenth_ws/src/f1tenth_gym_ros/maps/Catalunya_Obstacle'|g" ~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml
sed -i "s|map_img_ext:.*|map_img_ext: '.png'|g" ~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml
```
*(Verifica opcionalmente que en `sim.yaml` las coordenadas iniciales `sx` y `sy` correspondan a una zona despejada de la pista).*

### 3.5. Instalar Dependencias de ROS 2 y Compilar
Regresa a la raíz de tu espacio de trabajo, instala las dependencias omitiendo la clave obsoleta `python-transforms3d-pip` (ya instalada vía `pip` en el paso 3.2) y compila:

```bash
source /opt/ros/humble/setup.bash
cd ~/f1tenth_ws
rosdep install -i --from-path src --rosdistro humble --skip-keys python-transforms3d-pip -y
colcon build --symlink-install
source install/setup.bash
```

### 3.6. Ejecución de la Simulación
1. **Terminal 1 (Simulador y RViz):** Lanza el entorno gráfico y el puente de físicas:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/f1tenth_ws/install/setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```
2. **Terminal 2 (Controlador Reactivo):** Abre una nueva terminal, carga el entorno y ejecuta el nodo:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/f1tenth_ws/install/setup.bash
   ros2 run controllers gap_node
   ```
   *(Alternativamente, puedes ejecutarlo directamente con `python3 ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/src/controllers/controllers/gap_node.py`)*.

Verás que el vehículo comienza a procesar los datos del escáner láser, esquivando los obstáculos de la pista, mientras la terminal imprime los tiempos limpios de cada vuelta completada.

---

## 4. Parámetros de Calibración Clave (Tuning)

Si deseas probar el controlador en otras pistas o modificar la agresividad de la conducción, puedes ajustar las siguientes variables críticas directamente dentro del script `gap_node.py`:

| Parámetro | Valor Óptimo | Descripción Técnica |
| :--- | :--- | :--- |
| **`bubble_radius`** | `0.60 m` | Radio de seguridad para aislar obstáculos críticos y evitar colisiones frontales o raspones laterales con los muros. |
| **`np.clip` (LiDAR)** | `3.9 m` | Distancia máxima de visión para prevenir cortes de vértice (*corner-cutting*) y forzar al coche a mantenerse centrado. |
| **Velocidad Recta** | `14.0 m/s` | Velocidad máxima en rectas largas con tracción despejada. El sistema la escalona automáticamente usando *Trail Braking*. |
| **Campo de Visión** | `±70°` | Ángulo de apertura útil del LiDAR. Ignora los muros laterales y traseros para evitar que el coche tome decisiones erráticas. |

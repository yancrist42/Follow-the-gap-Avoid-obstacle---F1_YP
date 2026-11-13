# F1Tenth Autonomous Racing: Controlador Reactivo "Follow the Gap" con Telemetría y Trail Braking

Este repositorio contiene una implementación avanzada del algoritmo clásico **Follow-the-Gap (FtG)** optimizada para pruebas de resistencia (*Endurance*) en el simulador **F1Tenth**, utilizando **ROS 2 Humble**. 

El sistema combina navegación reactiva basada en LiDAR con un perfil de velocidad tipo *Trail Braking* y un módulo integrado de telemetría por odometría para el registro automático de tiempos por vuelta. Además, incluye mapas personalizados (Circuito de Catalunya) con obstáculos estáticos para poner a prueba la evasión de colisiones y el tiempo de reacción.

*(Arrastra y suelta aquí un GIF o video corto de tu simulación funcionando)*

---

## 🏎️ 1. Arquitectura de Control y Enfoque Teórico

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

## 📂 2. Estructura del Código

El núcleo funcional del controlador reside en el archivo `gap_node.py`. Este script define la clase `ReactiveFollowGap`, un nodo de ROS 2 que interactúa con el motor de físicas mediante:

* **Suscriptores:**
  * `/scan` (`sensor_msgs/msg/LaserScan`): Recibe las lecturas del entorno a 270 grados y alimenta la función `lidar_callback`.
  * `/ego_racecar/odom` (`nav_msgs/msg/Odometry`): Rastrea la posición espacial a través de la función `odom_callback` para el cronometraje de vueltas.
* **Publicador:**
  * `/drive` (`ackermann_msgs/msg/AckermannDriveStamped`): Envía los comandos calculados de aceleración y ángulo de dirección a los actuadores del simulador.
* **Funciones Clave:**
  * `preprocess_lidar`: Filtra los ángulos muertos y recorta el rango efectivo.
  * `find_max_gap` & `find_best_point`: Analizan la matriz de espacio libre para determinar la trayectoria.

---

## 🛠️ 3. Guía de Instalación y Uso (Desde Cero)

Esta guía detalla los pasos para instalar el simulador base de F1Tenth desde cero, integrar este controlador reactivo y ejecutar la simulación en el mapa personalizado de Catalunya.

### 3.1. Requisitos Previos
* Sistema Operativo: Ubuntu 22.04
* ROS 2 Humble instalado y configurado (`source /opt/ros/humble/setup.bash`).
* Python 3 y herramientas de compilación de ROS (`colcon`, `rosdep`).

### 3.2. Crear el Entorno e Instalar el Simulador Base
Abre una terminal e instala la librería del motor de físicas del simulador. Luego, crea un nuevo espacio de trabajo (*workspace*) y clona el repositorio oficial de F1Tenth:

```bash
# Instalar el motor de físicas de Python
pip3 install f1tenth-gym

# Crear el espacio de trabajo
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# Clonar el simulador oficial de F1Tenth para ROS 2
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
```

### 3.3. Clonar este Controlador Reactivo
En la misma carpeta `src`, clona este repositorio para agregar el código del algoritmo *Follow the Gap* y los mapas personalizados:
```bash
cd ~/f1tenth_ws/src
git clone https://github.com/tu_usuario/Follow-the-gap-Avoid-obstacle---F1_YP.git
```

### 3.4. Configurar el Mapa Personalizado
Para que el simulador reconozca los obstáculos, debemos trasladar los mapas al paquete principal:
1. Copia los archivos del circuito de Catalunya mediante la terminal:
   ```bash
   cp ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/maps/Catalunya* ~/f1tenth_ws/src/f1tenth_gym_ros/maps/
   ```
2. Edita el archivo de configuración del simulador para que cargue la nueva pista. Abre `~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml` y cambia la ruta de `map_path` a la siguiente:
   ```yaml
   map_path: '/home/TU_USUARIO/f1tenth_ws/src/f1tenth_gym_ros/maps/Catalunya_Obstacle'
   ```
   *(Asegúrate de reemplazar `TU_USUARIO` por tu nombre de usuario de Ubuntu. No incluyas las extensiones .png o .yaml al final de la ruta).*

### 3.5. Instalar Dependencias y Compilar
Regresa a la raíz de tu espacio de trabajo, resuelve las dependencias de ROS 2 y compila el entorno:
```bash
cd ~/f1tenth_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

### 3.6. Ejecución de la Simulación
Primero, lanza el entorno gráfico. Esto abrirá RViz mostrando la pista y los muros físicos:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Abre una **nueva terminal** (dejando la anterior en ejecución), carga las variables de entorno de ROS 2 y ejecuta el controlador reactivo:
```bash
source ~/f1tenth_ws/install/setup.bash
cd ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/src/controllers/controllers
python3 gap_node.py
```
Verás que el vehículo comienza a procesar los datos del escáner láser, esquivando los obstáculos de la pista, mientras la terminal imprime los tiempos limpios de cada vuelta completada.

---

## 📊 4. Parámetros de Calibración Clave (Tuning)

Si deseas probar el controlador en otras pistas o modificar la agresividad de la conducción, puedes ajustar las siguientes variables críticas directamente dentro del script `gap_node.py`:

| Parámetro | Valor Óptimo | Descripción Técnica |
| :--- | :--- | :--- |
| **`bubble_radius`** | `0.60 m` | Radio de seguridad para aislar obstáculos críticos y evitar colisiones frontales o raspones laterales con los muros. |
| **`np.clip` (LiDAR)** | `3.9 m` | Distancia máxima de visión para prevenir cortes de vértice (*corner-cutting*) y forzar al coche a mantenerse centrado. |
| **Velocidad Recta** | `14.0 m/s` | Velocidad máxima en rectas largas con tracción despejada. El sistema la escalona automáticamente usando *Trail Braking*. |
| **Campo de Visión** | `±70°` | Ángulo de apertura útil del LiDAR. Ignora los muros laterales y traseros para evitar que el coche tome decisiones erráticas. |

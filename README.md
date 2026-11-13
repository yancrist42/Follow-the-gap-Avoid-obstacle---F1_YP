# Controlador Reactivo "Follow the Gap" para F1Tenth en ROS 2

Este repositorio contiene la implementación de un controlador reactivo autónomo basado en el algoritmo **Follow the Gap**, diseñado para el simulador F1Tenth utilizando ROS 2 Humble. Además, incluye mapas personalizados (Circuito de Catalunya) con obstáculos estáticos para poner a prueba la evasión de colisiones y el tiempo de reacción.

*(Arrastra y suelta aquí un GIF o video corto de tu simulación funcionando)*

---

## 1. Enfoque Teórico: Algoritmo Follow the Gap

El algoritmo **Follow the Gap** es un método de navegación reactiva que permite a un vehículo autónomo esquivar obstáculos en tiempo real sin depender de la planificación global o waypoints predefinidos. El proceso lógico que ejecuta el nodo iterativamente es el siguiente:

1. **Preprocesamiento del LiDAR:** Se reciben los datos del escáner láser y se filtran los valores infinitos o fuera de rango para evitar errores matemáticos.
2. **Detección y Burbuja de Seguridad:** Se identifica el punto (obstáculo) más cercano al vehículo. Para evitar rozar las paredes, se establece un "radio" de seguridad alrededor de este punto (burbuja) y se asume temporalmente que esa zona es de distancia cero.
3. **Identificación del Hueco (Gap):** El algoritmo busca la secuencia consecutiva más larga de rayos libres (distancias no nulas) en el campo de visión.
4. **Selección del Objetivo:** Dentro del hueco más grande encontrado, se identifica el punto más lejano y se calcula el ángulo hacia esa dirección segura.
5. **Control Cinemático:** Se determina la dirección final de las ruedas y se ajusta la velocidad longitudinal dinámicamente en función del ángulo de giro (más lento en curvas cerradas, más rápido en rectas).

---

## 2. Estructura del Código

El núcleo funcional del controlador reside en el archivo `gap_node.py`. Este script define un nodo de ROS 2 (`gap_node`) que interactúa directamente con el motor de físicas de la simulación mediante:

* **Suscripción (`/scan`):** 
  * Tipo de mensaje: `sensor_msgs/msg/LaserScan`
  * Función: Recibe las lecturas del entorno a 270 grados y alimenta la función de callback principal.
* **Publicación (`/drive`):** 
  * Tipo de mensaje: `ackermann_msgs/msg/AckermannDriveStamped`
  * Función: Envía los comandos calculados de aceleración (`speed`) y ángulo de dirección (`steering_angle`) a los actuadores del simulador.

---

## 3. Guía de Instalación y Uso

> **Nota:** Estas instrucciones asumen que ya tienes instalado ROS 2 Humble y posees el simulador base `f1tenth_gym_ros` compilado en tu propio espacio de trabajo (*workspace*).

### 3.1. Clonar el Controlador
Abre una terminal, dirígete a la carpeta `src` de tu espacio de trabajo y clona este repositorio para integrarlo con tu simulador:
```bash
cd ~/tu_espacio_de_trabajo/src
git clone https://github.com/tu_usuario/Follow-the-gap-Avoid-obstacle---F1_YP.git
```

### 3.2. Configurar el Mapa Personalizado
Para probar la evasión de colisiones, debes integrar los mapas de este repositorio al simulador base:
1. Copia los archivos `Catalunya_map.png` y `Catalunya_Obstacle.yaml` desde la carpeta `maps/` de este repositorio hacia la carpeta de tu simulador: `.../src/f1tenth_gym_ros/maps/`.
2. Abre el archivo de configuración `sim.yaml` (en la carpeta `config/` del simulador) y modifica la ruta del mapa para que cargue el circuito con obstáculos:
   ```yaml
   map_path: '/home/tu_usuario/tu_espacio_de_trabajo/src/f1tenth_gym_ros/maps/Catalunya_Obstacle'
   ```
   *(Asegúrate de no incluir las extensiones .png o .yaml al final de la ruta).*

### 3.3. Compilar el Espacio de Trabajo
Regresa a la raíz de tu espacio de trabajo y compila el entorno para registrar el nuevo paquete de controladores:
```bash
cd ~/tu_espacio_de_trabajo
colcon build --symlink-install
source install/setup.bash
```

### 3.4. Ejecución del Nodo
Primero, lanza el entorno de simulación. Esto abrirá RViz mostrando el mapa físico con los muros:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Abre una **nueva terminal** (dejando la anterior en ejecución), carga las variables de entorno de ROS 2 y ejecuta tu controlador reactivo:
```bash
source ~/tu_espacio_de_trabajo/install/setup.bash
cd ~/tu_espacio_de_trabajo/src/Follow-the-gap-Avoid-obstacle---F1_YP/src/controllers/controllers
python3 gap_node.py
```
El vehículo comenzará a procesar los datos del LiDAR y se moverá de forma autónoma evadiendo los obstáculos de la pista.

---

## 4. Parámetros Ajustables (Tuning)

Si deseas probar el controlador en otras pistas o modificar la agresividad de la conducción, puedes ajustar las siguientes variables críticas directamente dentro del script `gap_node.py`:

* **Velocidad de Desplazamiento (`velocity`):** Se encuentra en la sección final de cálculo. Un valor entre `1.0` y `2.0` m/s es seguro. Si excedes el límite físico, el coche podría no frenar a tiempo debido a la latencia natural de respuesta.
* **Radio de la Burbuja de Seguridad:** Determina cuántos índices del arreglo del LiDAR adyacentes al obstáculo más cercano se igualarán a cero. Un radio mayor genera sobreviraje para alejarse excesivamente de las paredes; un radio pequeño permite carreras ajustadas pero aumenta el riesgo de colisión frontal.
* **Campo de Visión Útil:** Se recomienda delimitar la lectura del arreglo del LiDAR a los ángulos frontales (ej. -90° a +90°) descartando las lecturas traseras. Esto previene que el coche retroceda o gire erráticamente al detectar espacios libres a sus espaldas.

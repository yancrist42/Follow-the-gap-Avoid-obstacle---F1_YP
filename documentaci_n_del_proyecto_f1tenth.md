# F1TENTH Controlador Autónomo: Reactive Follow-the-Gap & Telemetría

Este repositorio contiene la implementación de un nodo avanzado de control reactivo basado en el algoritmo **Follow-the-Gap (FtG)**, diseñado para pruebas de resistencia (Endurance) en el simulador F1TENTH con ROS 2 Humble.

**Características del Controlador Reactivo**

* **Navegación estable a alta velocidad:** Capaz de completar más de 10 vueltas consecutivas sin colisiones a un promedio de \~42 segundos por vuelta.

* **Frenado dinámico (Trail Braking):** Desaceleración escalonada (14.0 m/s a 6.0 m/s) para evitar derrapes y pérdida de tracción en curvas cerradas.

* **Sistema de Telemetría Integrado:** Utiliza la odometría del vehículo para fijar una línea de meta automática y registrar los tiempos de cada vuelta en la consola de manera limpia.

* **Evasión optimizada:** Campo de visión (FoV) filtrado a ±70° y recorte de distancia a 3.9 metros para evitar cortes de vértice (*corner-cutting*).

## 1. Estructura y Ubicación del Controlador

Este nodo fue desarrollado para integrarse directamente en el paquete `controllers` del repositorio principal de F1TENTH.

El archivo principal, `gap_node.py`, contiene toda la lógica geométrica y el sistema de cronometraje. Debe estar ubicado en la siguiente ruta dentro de tu espacio de trabajo:

`/home/tu_usuario/F1Tenth-Repository/src/controllers/controllers/gap_node.py`

*(Asegúrate de reemplazar `tu_usuario` por el nombre de usuario de tu máquina, por ejemplo, `yancrist`)*.

## 2. Instrucciones de Ejecución

A continuación, se detallan los pasos exactos para compilar el espacio de trabajo, lanzar el entorno virtual y ejecutar el controlador autónomo.

### 2.1. Compilar el espacio de trabajo (Workspace)

Antes de ejecutar el nodo por primera vez o después de hacerle modificaciones al código, es obligatorio compilar el repositorio. Abre una terminal y ejecuta:

```
cd ~/F1Tenth-Repository
colcon build

```

Una vez terminada la compilación, refrescamos el entorno con el siguiente comando:

```
source install/setup.bash

```

### 2.2. Iniciar el simulador

Para cargar la pista, el modelo del robot y la interfaz de RViz, utiliza el archivo de lanzamiento (launch file) oficial del simulador:

```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

```

*(Nota: La primera vez que ejecutes este comando, el simulador puede tardar unos segundos en cargar el entorno gráfico).*

🎥 **Demostración: Lanzamiento del Simulador**

*(Arrastra y suelta aquí tu video corto mostrando cómo se abre RViz con el mapa)*

### 2.3. Ejecutar el nodo Follow-the-Gap

Con el simulador corriendo, abre una **nueva terminal** (manteniendo la anterior abierta).

Primero, asegúrate de refrescar las variables de entorno de ROS 2 en esta nueva pestaña y navega hasta la carpeta exacta donde se encuentra el script:

```
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository/src/controllers/controllers

```

Finalmente, ejecuta el script de Python:

```
python3 gap_node.py

```

En la terminal verás el mensaje `"🏁 LÍNEA DE META FIJADA. ¡Cronómetro iniciado!"` indicando que el sistema de telemetría está activo y el coche comenzará a moverse.

🎥 **Demostración: Ejecución del Nodo y Telemetría**

*(Arrastra y suelta aquí tu video mostrando cómo ejecutas el comando y cómo la terminal imprime los tiempos de vuelta mientras el coche corre)*

## 3. Tópicos de ROS 2 Utilizados

Durante la ejecución de este nodo, el controlador interactúa principalmente con tres tópicos fundamentales del simulador:

* **`/scan` (Suscripción):** Recibe mensajes de tipo `LaserScan`. Proporciona el arreglo de distancias y ángulos del entorno. El nodo filtra estos datos descartando los puntos traseros y laterales irrelevantes.

* **`/ego_racecar/odom` (Suscripción):** Recibe mensajes de tipo `Odometry`. Se extraen las coordenadas espaciales `(x, y)` para que el cronómetro detecte cuándo el vehículo se aleja más de 10 metros y cuándo vuelve a cruzar la zona de meta (radio de 4.5 metros).

* **`/drive` (Publicación):** Envía mensajes de tipo `AckermannDriveStamped`. Transmite al simulador el ángulo de dirección calculado (`steering_angle`) con un límite mecánico de ±0.4 radianes, y la velocidad de empuje (`speed`) basada en el perfil de Trail Braking.

## 4. Parámetros de Calibración Clave

Si deseas experimentar o adaptar este nodo a otros mapas (como un circuito con rectas más largas), puedes modificar los siguientes parámetros directamente en `gap_node.py`:

| Parámetro | Valor Actual | Explicación Física | 
| ----- | ----- | ----- | 
| `bubble_radius` | `0.60` metros | Tamaño de la burbuja que rodea al obstáculo más cercano. A velocidades >12 m/s, se requiere al menos 0.60m para evitar roces contra el muro debido al subviraje por inercia. | 
| `np.clip` max | `3.9` metros | Distancia de *Lookahead*. Valores mayores hacen que el coche corte curvas cerradas. Valores menores provocan movimientos erráticos en rectas. | 
| Velocidad Punta | `14.0` m/s | Se activa únicamente cuando el ángulo de dirección es `< 0.05` radianes y la pista libre frontal es `> 9.0` metros. | 

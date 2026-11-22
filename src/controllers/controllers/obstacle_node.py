import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry  # <-- Importamos el mensaje de Odometría


class ReactiveFollowGap(Node):
    """
    Implementación de Follow the Gap para evasión reactiva de obstáculos
    con rebase suave en rectas y Telemetría de Vueltas.
    """

    def __init__(self):
        super().__init__('reactive_node')
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        odom_topic = '/ego_racecar/odom'

        # Suscriptor al LIDAR y publicador de comandos Ackermann
        self.subscription = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )
        # Suscriptor a la Odometría para contar las vueltas
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Radio base de la "burbuja" de seguridad en metros
        self.bubble_radius = 0.65

        # Parámetros configurables por remapping/CLI
        self.declare_parameter('max_speed', 10.0)
        self.declare_parameter('car_label', 'EGO')
        self.max_speed = self.get_parameter('max_speed').value
        self.car_label = self.get_parameter('car_label').value

        # Estado para suavizado de dirección y rebase suave
        self.prev_steering_angle = 0.0
        self.max_steer_delta = 0.08  # Máximo cambio de ángulo por ciclo para evitar tirones
        self.steering_alpha = 0.65   # Peso del nuevo ángulo en el filtro suavizador

        # --- Variables del Sistema de Telemetría (Cronómetro) ---
        self.start_x = None
        self.start_y = None
        self.lap_start_time = None
        self.lap_count = 0
        self.is_away = False  # Bandera para saber si el coche ya salió de la línea de meta

    # ==========================================
    # SISTEMA DE TELEMETRÍA (SIN PREFIJOS DE ROS)
    # ==========================================
    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # 1. Fijar el punto de aparición
        if self.start_x is None:
            self.start_x = current_x
            self.start_y = current_y
            self.lap_start_time = self.get_clock().now()
            print("🏁 LÍNEA DE META FIJADA. ¡Cronómetro iniciado!")
            return

        dist_to_start = math.sqrt((current_x - self.start_x) ** 2 + (current_y - self.start_y) ** 2)

        # 2. Armar el sensor
        if dist_to_start > 10.0 and not self.is_away:
            self.is_away = True
            print("🔵 Vehículo en pista profunda. Sensor de meta armado.")

        # 3. Vuelta completada
        if dist_to_start < 4.5 and self.is_away:
            current_time = self.get_clock().now()
            lap_duration = (current_time - self.lap_start_time).nanoseconds / 1e9
            self.lap_count += 1

            # Textos completamente limpios en la terminal
            print(f"🟩 --- VUELTA {self.lap_count} COMPLETADA ---")
            print(f"⏱️ Tiempo: {lap_duration:.3f} segundos\n")

            self.lap_start_time = current_time
            self.is_away = False

    # ==========================================
    # ALGORITMO DE CONDUCCIÓN (FOLLOW THE GAP)
    # ==========================================
    def preprocess_lidar(self, ranges, angle_min, angle_inc):
        proc_ranges = np.array(ranges, dtype=np.float64)
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=6.5, neginf=0.0)
        # Mayor alcance (6.5 m) para anticipar obstáculos en rectas con tiempo
        proc_ranges = np.clip(proc_ranges, 0.0, 3.9)

        start_angle = -np.pi * (70.0 / 180.0)
        end_angle = np.pi * (70.0 / 180.0)
        start_i = max(0, int((start_angle - angle_min) / angle_inc))
        end_i = min(len(proc_ranges) - 1, int((end_angle - angle_min) / angle_inc))
        proc_ranges[:start_i] = 0.0
        proc_ranges[end_i + 1:] = 0.0
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        mask = free_space_ranges > 0.01
        max_gap_start, max_gap_end = 0, 0
        current_start, current_length, max_length = -1, 0, 0

        for i in range(len(mask)):
            if mask[i]:
                if current_start == -1:
                    current_start = i
                current_length += 1
            else:
                if current_length > max_length:
                    max_length = current_length
                    max_gap_start = current_start
                    max_gap_end = i - 1
                current_start = -1
                current_length = 0

        if current_length > max_length:
            max_gap_start = current_start
            max_gap_end = len(mask) - 1

        return max_gap_start, max_gap_end

    def find_best_point(self, start_i, end_i, ranges):
        gap_ranges = np.array(ranges[start_i:end_i + 1])
        if len(gap_ranges) == 0:
            return start_i
        max_val = np.max(gap_ranges)
        # Tolerancia del 5% del valor máximo para preferir un punto centrado y estable
        candidate_indices = np.where(gap_ranges >= max_val * 0.95)[0]
        best_local_i = candidate_indices[len(candidate_indices) // 2]
        return start_i + best_local_i

    def smooth_steering(self, target_angle):
        # 1. Suavizado exponencial
        blended = (self.steering_alpha * target_angle) + ((1.0 - self.steering_alpha) * self.prev_steering_angle)
        # 2. Límite de cambio por ciclo para evitar movimientos bruscos al rebasar
        delta = np.clip(blended - self.prev_steering_angle, -self.max_steer_delta, self.max_steer_delta)
        smoothed = np.clip(self.prev_steering_angle + delta, -0.4, 0.4)
        self.prev_steering_angle = smoothed
        return smoothed

    def lidar_callback(self, data):
        if data.angle_increment <= 0.0 or len(data.ranges) == 0:
            return

        # 1. Preprocesamiento
        proc_ranges = self.preprocess_lidar(data.ranges, data.angle_min, data.angle_increment)

        # 2. Obstáculo más cercano
        valid_indices = np.where(proc_ranges > 0.0)[0]
        if len(valid_indices) == 0:
            return
        closest_idx = valid_indices[np.argmin(proc_ranges[valid_indices])]
        closest_dist = proc_ranges[closest_idx]

        # 3. Eliminar puntos dentro de la burbuja (con margen extra si el obstáculo está cerca)
        dynamic_bubble = self.bubble_radius + (0.15 if closest_dist < 2.2 else 0.0)
        theta = np.arctan2(dynamic_bubble, max(closest_dist, 0.1))
        num_indices = int(theta / data.angle_increment)
        b_start = max(0, closest_idx - num_indices)
        b_end = min(len(proc_ranges) - 1, closest_idx + num_indices)
        proc_ranges[b_start:b_end + 1] = 0.0

        # 4. Encontrar la brecha máxima
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        if gap_start == 0 and gap_end == 0:
            return

        # 5. Encontrar el mejor punto
        best_idx = self.find_best_point(gap_start, gap_end, proc_ranges)

        # 6. Calcular ángulo, suavizar y aplicar límite mecánico
        raw_steering_angle = data.angle_min + (best_idx * data.angle_increment)
        clamped_target = np.clip(raw_steering_angle, -0.4, 0.4)
        steering_angle = self.smooth_steering(clamped_target)

        # Leer distancia frontal en un cono central limpio
        center_idx = len(data.ranges) // 2
        cone_half = max(1, int(np.radians(8.0) / data.angle_increment))
        c_start = max(0, center_idx - cone_half)
        c_end = min(len(data.ranges), center_idx + cone_half + 1)
        front_slice = np.array(data.ranges[c_start:c_end], dtype=np.float64)
        front_slice = front_slice[np.isfinite(front_slice) & (front_slice > 0.05)]
        front_dist = np.min(front_slice) if len(front_slice) > 0 else 10.0

        # 7. Control de velocidad: Trail Braking escalado a max_speed
        if abs(steering_angle) < 0.05:
            if front_dist > 9.0:
                speed = self.max_speed
            elif front_dist > 6.0:
                speed = min(9.5, self.max_speed * 0.85)
            elif front_dist > 4.0:
                speed = min(7.5, self.max_speed * 0.70)
            else:
                speed = min(6.0, self.max_speed * 0.55)
        elif abs(steering_angle) < 0.17:
            speed = min(7.0, self.max_speed * 0.65)
        elif abs(steering_angle) < 0.34:
            speed = min(5.0, self.max_speed * 0.50)
        elif abs(steering_angle) < 0.52:
            speed = min(3.5, self.max_speed * 0.35)
        else:
            speed = min(1.2, self.max_speed * 0.25)

        # Moderar velocidad al rebasar o maniobrar cerca de un obstáculo
        if closest_dist < 1.65 and abs(steering_angle) > 0.08:
            speed = min(speed, 4.5)
        if front_dist < 2.2:
            speed = min(speed, max(2.0, front_dist * 1.8))

        # 8. Publicar comando
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    reactive_node = ReactiveFollowGap()
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        # Mensaje limpio al presionar Ctrl+C
        print("🛑 Simulación detenida por el usuario. Cerrando telemetría...")
    finally:
        # Destruye el nodo de forma segura sin mostrar errores de código
        reactive_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
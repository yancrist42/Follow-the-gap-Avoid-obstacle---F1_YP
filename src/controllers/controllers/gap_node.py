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
    con Telemetría de Vueltas.
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
            10)
            
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10)
            
        # NUEVO: Suscriptor a la Odometría para contar las vueltas
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)

        # Radio de la "burbuja" de seguridad en metros
        self.bubble_radius = 0.60
        
        # --- Variables del Sistema de Telemetría (Cronómetro) ---
        self.start_x = None
        self.start_y = None
        self.lap_start_time = None
        self.lap_count = 0
        self.is_away = False  # Bandera para saber si el coche ya salió de la línea de meta

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

        dist_to_start = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)

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
            print(f"⏱️ Tiempo: {lap_duration:.3f} segundos\n") # El \n da un pequeño salto de línea extra
            
            self.lap_start_time = current_time
            self.is_away = False

    # ==========================================
    # ALGORITMO DE CONDUCCIÓN (FOLLOW THE GAP)
    # ==========================================
    def preprocess_lidar(self, ranges, angle_min, angle_inc):
        proc_ranges = np.array(ranges)
        proc_ranges = np.clip(proc_ranges, 0.0, 3.9)
        
        start_angle = -np.pi * (70.0 / 180.0)
        end_angle = np.pi * (70.0 / 180.0)
        
        start_i = max(0, int((start_angle - angle_min) / angle_inc))
        end_i = min(len(proc_ranges)-1, int((end_angle - angle_min) / angle_inc))
        
        proc_ranges[:start_i] = 0.0
        proc_ranges[end_i+1:] = 0.0
        
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
        gap_ranges = np.array(ranges[start_i:end_i+1])
        max_val = np.max(gap_ranges)
        max_indices = np.where(gap_ranges == max_val)[0]
        best_local_i = max_indices[len(max_indices) // 2]
        return start_i + best_local_i

    def lidar_callback(self, data):
        # 1. Preprocesamiento
        proc_ranges = self.preprocess_lidar(data.ranges, data.angle_min, data.angle_increment)
        
        # 2. Obstáculo más cercano
        valid_indices = np.where(proc_ranges > 0.0)[0]
        if len(valid_indices) == 0:
            return 
            
        closest_idx = valid_indices[np.argmin(proc_ranges[valid_indices])]
        closest_dist = proc_ranges[closest_idx]
        
        # 3. Eliminar puntos dentro de la burbuja
        theta = np.arctan2(self.bubble_radius, closest_dist)
        num_indices = int(theta / data.angle_increment)
        b_start = max(0, closest_idx - num_indices)
        b_end = min(len(proc_ranges) - 1, closest_idx + num_indices)
        proc_ranges[b_start:b_end+1] = 0.0
        
        # 4. Encontrar la brecha máxima
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        if gap_start == 0 and gap_end == 0: 
            return 
            
        # 5. Encontrar el mejor punto
        best_idx = self.find_best_point(gap_start, gap_end, proc_ranges)
        
        # 6. Calcular ángulo y límite mecánico
        raw_steering_angle = data.angle_min + (best_idx * data.angle_increment)
        steering_angle = np.clip(raw_steering_angle, -0.4, 0.4)
        
        # Leer el promedio de los 10 láseres frontales
        center_idx = len(data.ranges) // 2
        front_dist = np.mean(data.ranges[center_idx-5:center_idx+5])
        
        # 7. Control de velocidad: Trail Braking 
        if abs(steering_angle) < 0.05:      
            if front_dist > 9.0:
                speed = 14.0                
            elif front_dist > 6.0:
                speed = 9.5                 
            elif front_dist > 4.0:
                speed = 7.5                 
            else:
                speed = 6.0                 
        elif abs(steering_angle) < 0.17:    
            speed = 7.0                     
        elif abs(steering_angle) < 0.34:    
            speed = 5.0                     
        elif abs(steering_angle) < 0.52:    
            speed = 3.5                     
        else:                               
            speed = 1.2                     
            
        # 8. Publicar comando
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
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

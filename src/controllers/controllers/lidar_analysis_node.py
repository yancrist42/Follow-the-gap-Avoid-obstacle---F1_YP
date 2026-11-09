import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class LidarAnalysisNode(Node):
    def __init__(self):
        super().__init__('lidar_analysis_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.last_time = time.time()
        self.msg_count = 0

    def scan_callback(self, msg):
        self.msg_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.last_time

        # Se ejecuta cada 1 segundo para promediar la frecuencia y no saturar la consola
        if elapsed_time >= 1.0:
            frequency = self.msg_count / elapsed_time
            angular_res_deg = math.degrees(msg.angle_increment)

            front_ranges = []
            rear_ranges = []
            
            current_angle = msg.angle_min
            for r in msg.ranges:
                # Convertir a grados y normalizar al rango 0° - 360°
                angle_deg = math.degrees(current_angle) % 360
                
                if 0 <= angle_deg <= 180:
                    front_ranges.append(r)
                else:
                    rear_ranges.append(r)
                    
                current_angle += msg.angle_increment

            # Extraer los primeros 3 valores y darles formato de dos decimales
            front_sample = front_ranges[:3]
            rear_sample = rear_ranges[:3]
            front_str = f"[{', '.join(f'{x:.2f}' for x in front_sample)}, ...]"
            rear_str = f"[{', '.join(f'{x:.2f}' for x in rear_sample)}, ...]"

            # Imprimir exactamente el formato solicitado
            self.get_logger().info(f"Resolución angular del LiDAR: {angular_res_deg:.3f} grados")
            self.get_logger().info(f"Frecuencia del LiDAR: {frequency:.3f} Hz")
            self.get_logger().info(f"Cantidad de mediciones frontales (0° a 180°): {len(front_ranges)}")
            self.get_logger().info(f"Cantidad de mediciones traseras (180° a 360°): {len(rear_ranges)}")
            self.get_logger().info(f"Primeros valores frontales: {front_str}")
            self.get_logger().info(f"Primeros valores traseros: {rear_str}\n")

            # Reiniciar contadores para el siguiente bloque de tiempo
            self.msg_count = 0
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = LidarAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

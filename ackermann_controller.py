import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math

class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        
        # Suscriptor a odometría del simulador F1Tenth
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        # Publicador de comandos Ackermann
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
            
        # Variables de la máquina de estados
        self.state = 'STRAIGHT'
        self.corner_index = 0
        
        # 1. Definir coordenadas de pre-esquina (X, Y) y ángulo de salida (Yaw)
        # IMPORTANTE: Estos son valores de ejemplo. Debes medir en RViz o imprimir
        # tu odometría para colocar las coordenadas reales de la pista Levine.
        self.corners = [
            {'turn_x': 9.50, 'turn_y': 0.0, 'target_yaw': 1.57, 'steer': 0.35},  # Esquina 1 (90°)
            {'turn_x': 9.50, 'turn_y': 8.3, 'target_yaw': 3.14, 'steer': 0.45},  # Esquina 2 (180°)
            {'turn_x': -13.5,  'turn_y': 8.30, 'target_yaw': -1.57, 'steer': 0.35}, # Esquina 3 (-90°)
            {'turn_x': -13.5,  'turn_y': 0.0, 'target_yaw': 0.0,  'steer': 0.35}   # Esquina 4 (0°)
        ]
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.declare_parameter('speed_multiplier', 1.0)

    def odom_callback(self, msg):
        # Extraer coordenadas (x, y) de la odometría
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convertir el cuaternión de orientación a ángulo Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.control_loop()

    def control_loop(self):
        # Si ya pasó por todas las esquinas, detener el vehículo
        if self.corner_index >= len(self.corners):
            self.get_logger().info("Vuelta completada. Deteniendo el vehículo.", once=True)
            self.stop_vehicle()
            return

        target = self.corners[self.corner_index]
        drive_msg = AckermannDriveStamped()
        
        mult = self.get_parameter('speed_multiplier').value
	
        if self.state == 'STRAIGHT':
            # 2. Avanzar en línea recta
            drive_msg.drive.speed = 1.5* mult
            drive_msg.drive.steering_angle = 0.0
            
            # Calcular distancia entre el coche y el punto pre-esquina
            dist_to_corner = math.hypot(target['turn_x'] - self.current_x, target['turn_y'] - self.current_y)
            
            # Iniciar el giro suave 1 metro antes de la coordenada objetivo
            if dist_to_corner < 1.0:
                self.state = 'TURN'
                self.get_logger().info(f"Acercándose a esquina {self.corner_index + 1}. Iniciando giro.")
                
        elif self.state == 'TURN':
            # 3. Reducir velocidad y girar la dirección (steering_angle)
            drive_msg.drive.speed = 1.0
            drive_msg.drive.steering_angle = target['steer']
            
            # Calcular cuánto falta para alcanzar la orientación de la siguiente recta
            angle_diff = abs(self.normalize_angle(target['target_yaw'] - self.current_yaw))
            
            # 4. Finalizar giro cuando la orientación del coche coincida con la recta
            if angle_diff < 0.15: # Margen de tolerancia en radianes
                self.state = 'STRAIGHT'
                self.corner_index += 1
                self.get_logger().info("Giro finalizado. Avanzando en la siguiente recta.")
                
        # Publicar el mensaje ensamblado
        self.publisher.publish(drive_msg)

    def stop_vehicle(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.publisher.publish(drive_msg)

    def normalize_angle(self, angle):
        # Mantiene el ángulo en el rango de -pi a pi
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = AckermannController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

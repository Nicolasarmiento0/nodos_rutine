import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RutineAn4Node(Node):
    
    def __init__(self):
        super().__init__('rutine_an4_node')
        
        # Publicador para el comando de velocidad
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parámetros de movimiento
        self.move_distance = 0.5       # Distancia en metros
        self.speed = 0.2               # Velocidad en m/s
        self.movement_duration = self.move_distance / self.speed  # Tiempo necesario para recorrer la distancia
        self.timer = self.create_timer(1.0, self.start_autonomous_movement)
        self.get_logger().info("RutineAn4Node has been started.")
        
    def start_autonomous_movement(self):
        self.get_logger().info("Starting autonomous movement.")
        
        # Crear y publicar un mensaje Twist para mover el robot hacia adelante
        cmd_vel = Twist()
        cmd_vel.linear.x = self.speed
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Pausar el movimiento después del tiempo necesario
        time.sleep(self.movement_duration)

        # Detener el robot
        cmd_vel.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)
        self.get_logger().info("Movement completed. Robot stopped.")
        
        # Detener el temporizador para evitar que se repita el movimiento
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = RutineAn4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

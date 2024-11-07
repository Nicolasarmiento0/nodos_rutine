import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

#cambiar AutonomousMovementNode

class MovimientoAn4Node(Node):
    
    def __init__(self):
        super().__init__('movimiento_an4_node')
        
        # Publicador para el comando de velocidad
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parámetros de movimiento
        self.move_distance = 1.0       # Distancia en metros
        self.speed = 0.2               # Velocidad en m/s
        self.movement_duration = self.move_distance / self.speed  # Tiempo necesario para recorrer 1 metro a la velocidad establecida
        
        # Inicializar el estado de la secuencia de movimientos
        self.movement_sequence = ["forward", "backward", "left", "right"]
        self.current_step = 0
        
        # Temporizador para manejar la secuencia de movimientos
        self.timer = self.create_timer(1.0, self.perform_movement)

        self.get_logger().info("MovimientoAn4Node has been started.")
        
    def perform_movement(self):
        # Crear el mensaje Twist para definir el movimiento
        cmd_vel = Twist()
        
        # Seleccionar el movimiento en función de la secuencia
        direction = self.movement_sequence[self.current_step]
        
        if direction == "forward":
            cmd_vel.linear.x = self.speed
            self.get_logger().info("Moving forward.")
        
        elif direction == "backward":
            cmd_vel.linear.x = -self.speed
            self.get_logger().info("Moving backward.")
        
        elif direction == "left":
            cmd_vel.linear.y = self.speed  # Movimiento lateral hacia la izquierda
            self.get_logger().info("Moving left.")
        
        elif direction == "right":
            cmd_vel.linear.y = -self.speed  # Movimiento lateral hacia la derecha
            self.get_logger().info("Moving right.")
        
        # Publicar el comando de velocidad
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Esperar el tiempo necesario para completar el movimiento de un metro
        time.sleep(self.movement_duration)

        # Detener el robot después de completar el movimiento
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)
        self.get_logger().info("Stopping.")

        # Pasar al siguiente paso de la secuencia de movimiento
        self.current_step += 1

        # Verificar si completó la secuencia
        if self.current_step >= len(self.movement_sequence):
            self.get_logger().info("Movement sequence completed.")
            self.timer.cancel()  # Detener el temporizador para que no se repita

def main(args=None):
    rclpy.init(args=args)
    node = MovimientoAn4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

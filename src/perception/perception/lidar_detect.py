#Código para el nodo del LIDAR
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Int32

class LidarMonitor(Node):

    def __init__(self):
        super().__init__('lidar_monitor')
        
        self.safe_distance = 0.3        # [m]
        self.front_angle = math.radians(80)  # +-x grados
        
        self.obstacle_state = False     # estado confirmado
        self.obstacle_since = None      # cuándo empezó a verse obstáculo
        self.clear_since = None         # cuándo empezó a despejarse
        self.stuck_since = None

        self.T_detect = 0.3             # segundos para confirmar obstáculo
        self.T_clear = 0.5              # segundos para limpiar obstáculo
        self.T_stuck = 30.0             # segundos para determinar si el robot dejó de moverse por mucho tiempo
        
        self.r_sum_prev = 0.0
        
        #suscriptor
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.scan_callback,
            10
        )
        
        self.shutdown_sub = self.create_subscription(
            Bool,
            '/nav/execution_finalized',
            self.shutdown_callback,
            10
        )
        
        
        #Publicador        
        self.obstacle_pub = self.create_publisher(
            Int32, 
            '/nav/obstacle_dir', 
            10
        )
        
        self.stuck_pub = self.create_publisher(
            Bool,
            '/nav/stuck',
            10
        )
        
        self.get_logger().info("Nodo LidarMonitor iniciado")
        
    def shutdown_callback(self, msg):
        if msg.data == True:
            self.get_logger().info("Finalizando lidar...")
            self.destroy_node()
            rclpy.shutdown()
            return
    
    def scan_callback(self, msg: LaserScan):
        #Inicializar el reloj
        now = self.get_clock().now().nanoseconds * 1e-9
        
        message = Int32()
        
        r_sum = 0.0
        obstacle_detected = False
        left_detected = False
        center_detected = False
        right_detected = False

        angle = msg.angle_min

        for r in msg.ranges:
            r_sum += r
            # ¿Este rayo está dentro del frente?
            if abs(angle) <= self.front_angle:
                # Validar medición
                if math.isfinite(r):
                    if msg.range_min <= r <= msg.range_max:
                        if r < self.safe_distance:
                            obstacle_detected = True
                            
                            # Clasificación angular
                            if abs(angle) <= math.radians(30):
                                center_detected = True
                            elif angle > 0:
                                left_detected = True
                            else:
                                right_detected = True
                            break
            angle += msg.angle_increment
            
        if (r_sum * 0.98) <= self.r_sum_prev <= (r_sum * 1.02):
            if self.stuck_since is None:
                self.stuck_since = now
            elif now - self.stuck_since >= self.T_stuck:
                self.get_logger().warn("Stuck, killing process...")
                self.stuck_pub.publish(Bool(data=True))
        else:                    
            self.r_sum_prev = r_sum
            self.stuck_since = None
            self.stuck_pub.publish(Bool(data=False))
        
         # 2. Histéresis temporal
        # -------------------------
        if obstacle_detected:
            self.clear_since = None
            
            if not self.obstacle_state:
                
                if self.obstacle_since is None:
                    self.obstacle_since = now
                elif now - self.obstacle_since >= self.T_detect:
                    self.obstacle_state = True
                    
                    if center_detected:
                        obstacle_detected = True
                        self.get_logger().warn("Obstáculo al FRENTE")
                        message.data = 2

                    elif left_detected:
                        obstacle_detected = True
                        self.get_logger().warn("Obstáculo a la IZQUIERDA")
                        message.data = 1

                    elif right_detected:
                        obstacle_detected = True
                        self.get_logger().warn("Obstáculo a la DERECHA")
                        message.data = 3
                    
                    self.obstacle_pub.publish(message)
                    self.obstacle_since = None
        else:
            self.obstacle_since = None
            message.data = 0

            if self.obstacle_state:
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= self.T_clear:
                    self.obstacle_state = False
                    self.obstacle_pub.publish(message)
                    self.get_logger().info(f"Despejado: {message}")

def main():
    rclpy.init()
    node = LidarMonitor()
           
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado. Cerrando nodo...")

    except RuntimeError as e:
        if "Context must be initialized" in str(e):
            pass
        else:
            raise e


if __name__ == '__main__':
    main()


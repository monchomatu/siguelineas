#Código para el nodo del LIDAR
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class LidarMonitor(Node):

    def __init__(self):
        super().__init__('lidar_monitor')
        
        self.safe_distance = 0.25        # [m]
        self.front_angle = math.radians(70)  # +-x grados
        
        self.obstacle_state = False     # estado confirmado
        self.obstacle_since = None      # cuándo empezó a verse obstáculo
        self.clear_since = None         # cuándo empezó a despejarse

        self.T_detect = 0.3             # segundos para confirmar obstáculo
        self.T_clear = 0.5              # segundos para limpiar obstáculo
                
        #suscriptor
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.scan_callback,
            10
        )
        
        #Publicador
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/nav/obstacle_detected',
            10
        )
        
        self.get_logger().info("Nodo LidarMonitor iniciado")
    
    def scan_callback(self, msg: LaserScan):
        #Inicializar el reloj
        now = self.get_clock().now().nanoseconds * 1e-9

        obstacle_detected = False

        angle = msg.angle_min

        for r in msg.ranges:
            # ¿Este rayo está dentro del frente?
            if abs(angle) <= self.front_angle:

                # Validar medición
                if math.isfinite(r):
                    if msg.range_min <= r <= msg.range_max:
                        if r < self.safe_distance:
                            obstacle_detected = True
                            break

            angle += msg.angle_increment
        
         # 2. Histéresis temporal
        # -------------------------
        if obstacle_detected:
            self.clear_since = None

            if not self.obstacle_state:
                
                if self.obstacle_since is None:
                    self.obstacle_since = now
                elif now - self.obstacle_since >= self.T_detect:
                    self.obstacle_state = True
                    self.obstacle_pub.publish(Bool(data=True))
                    self.get_logger().warn("Obstáculo detectado")
                    self.obstacle_since = None
        else:
            self.obstacle_since = None

            if self.obstacle_state:
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= self.T_clear:
                    self.obstacle_state = False
                    self.obstacle_pub.publish(Bool(data=False))
                    self.get_logger().info("Despejado")

def main():
    rclpy.init()
    node = LidarMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


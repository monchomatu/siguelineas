import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math


class PathDraw(Node):

    def __init__(self):
        super().__init__('path_draw')
        
        # Crear loggers
        self.executed_path = Path()
        self.executed_path.header.frame_id = 'odom'
        self.last_pose = None
        
        #Crear una distancia mínima entre puntos para evitar ruido
        self.min_dist = 0.02
        
        # ===== Publisher =====
        self.pub_path = self.create_publisher(Path, '/executed_path', 10)
       
       # ===== Subscriber a odometría =====
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publicar periodicamente
        #self.timer = self.create_timer(0.1, self.publish_path)

        self.get_logger().info("Path Drawing inicializado")
        
    def odom_callback(self, msg: Odometry):
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # FIltro de distancia mínima
        if self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            if (dx*dx + dy*dy) < self.min_dist**2:
                return  
        
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'odom'
        
        
        # Guardamos las poses
        pose.pose.position = msg.pose.pose.position
        pose.pose.orientation = msg.pose.pose.orientation

        self.executed_path.poses.append(pose)
        self.executed_path.header.stamp = pose.header.stamp
        
        self.pub_path.publish(self.executed_path)
        
        if len(self.executed_path.poses) % 20 == 0:
            self.get_logger().info(
                f"{len(self.executed_path.poses)} puntos en trayectoria ejecutada"
            )
            
        self.last_pose = (x, y)

def main(args=None):
    rclpy.init(args=args)
    node = PathDraw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


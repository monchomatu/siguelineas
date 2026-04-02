import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import datetime
import matplotlib
matplotlib.use('Agg')   # backend sin GUI
import matplotlib.pyplot as plt
from pathlib import Path as dirpath
from path_makers.planning.Bit_Map import BitmapMap
import numpy as np

from ament_index_python.packages import get_package_share_directory
import os

class PathDraw(Node):

    def __init__(self):
        super().__init__('path_draw')
        
        
        self.declare_parameter("use_replan", True)
        self.use_replan = self.get_parameter("use_replan").value
        
        if self.use_replan:
            #Guardado de traycetorias en png
            self.results_dir = dirpath("results/executed_paths/with_replan")
        else:
            self.results_dir = dirpath("results/executed_paths/no_replan")
        
        
        self.used_map = np.zeros(300, dtype=np.uint8)
        self.half_size = 0
        self.tracked_x = []
        self.tracked_y = []
        
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
        
        self.shutdown_sub = self.create_subscription(
            Bool,
            '/nav/execution_finalized',
            self.shutdown_callback,
            10
        )
        
        
        self.tracked_sub = self.create_subscription(
            Path,
            '/tracked_path',
            self.path_callback,
            10
        )
        self.get_logger().info("Path Drawing inicializado")
        
    def shutdown_callback(self, msg):
        if msg.data == True:
            self.save_traj()
            self.get_logger().info("Finalizando path drawer...")
            self.destroy_node()
            rclpy.shutdown()
            return
    
    def path_callback(self, msg: Path):
        xs = []
        ys = []

        for pose in msg.poses:
            xs.append(pose.pose.position.x)
            ys.append(pose.pose.position.y)

        self.tracked_x = xs
        self.tracked_y = ys
        
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
        self.last_pose = (x, y)
    
    
    def save_traj(self):
        self.results_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"executed_path_{timestamp}.png"
        filepath = os.path.join(self.results_dir, filename)
        
        pkg_dir = get_package_share_directory('stage_utils')

        bitmap_path = os.path.join(
            pkg_dir,
            'world',
            'bitmaps',
            'cave.png'
        )

        bitmap = BitmapMap(bitmap_path, world_size=16.0)
        
        xs = []
        ys = []
        
        
        self.used_map, self.half_size = bitmap.give_map()
        
        for pose in self.executed_path.poses:
            xt = pose.pose.position.x
            yt = pose.pose.position.y
            
            xn, yn = self.transform_coordinates(xt, yt)
            
            xs.append(xn)
            ys.append(yn)
        
        tracked_xs = []
        tracked_ys = []
        
        for m, n in zip(self.tracked_x, self.tracked_y):
            
            tracked_xn, tracked_yn = self.transform_coordinates(m, n)
            
            tracked_xs.append(tracked_xn)
            tracked_ys.append(tracked_yn)
            
        
        plt.figure(figsize=(6,6))
        plt.imshow(self.used_map, cmap = "gray_r", origin = "upper", extent=[-self.half_size, self.half_size, -self.half_size, self.half_size,])
        plt.plot(xs, ys, label="Executed path", color = "red")
        
        plt.plot(tracked_xs, tracked_ys, label="Tracked path", color = "green")
        # Punto inicial
        plt.scatter(xs[0], ys[0], s=80, marker='o', color = "blue")

        # Punto final
        plt.scatter(xs[-1], ys[-1], s=80, marker='o', color = "blue")
        # Texto inicio
        plt.text(
            xs[0], ys[0],
            f"Start\n({xs[0]:.2f}, {ys[0]:.2f})",
            fontsize=8,
            ha='right'
        )

        # Texto final
        plt.text(
            xs[-1], ys[-1],
            f"Goal\n({xs[-1]:.2f}, {ys[-1]:.2f})",
            fontsize=8,
            ha='left'
        )
        
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Run Trajectories")
        plt.axis("equal")
        plt.xlim(-self.half_size, self.half_size)
        plt.ylim(-self.half_size, self.half_size)
        plt.legend()
        plt.savefig(filepath, dpi=300)
        plt.close()
        self.get_logger().warn("Publicando imagen de trayectoria")
    
    def transform_coordinates(self, x, y):
        theta = math.radians(45)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        tx = -7
        ty = -7
        
        xn = (cos_t * x - sin_t * y) + tx
        yn = (sin_t * x + cos_t * y) + ty
        
        return xn, yn
        

def main(args=None):
    rclpy.init(args=args)
    node = PathDraw()
    
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
    finally:
        print('Nodo Cerrado')


if __name__ == '__main__':
    main()


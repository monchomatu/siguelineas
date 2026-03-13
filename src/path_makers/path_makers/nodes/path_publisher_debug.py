import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Bool
import numpy as np

from path_makers.planning.rrt_exp import compute_rrt_path
from path_makers.planning.rrt_exp import map_path_to_odom


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        #definir si se busca un rrt o un debug
        self.mode = 'debug'
        
        # ===== Publisher =====
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
       
       # ===== Subscriber a odometría =====
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        #Subscriber a replan
        self.create_subscription(
            Bool,
            '/nav/replan_request',
            self.replan_cb,
            10
        )
        
        self.robot_map_pose = (-7, -7, np.pi/4)
        start_point = (-7, -7)
        self.goal_point = (6, 4)
        
        if self.mode == 'debug':
        
           self.path_map, self.final_path = self.generate_test_path()
            
        elif self.mode == 'rrt':
        
            self.path_map, self.final_path = self.generate_rrt_path(start_point)
            

       
        # Publicar periodicamente
        self.timer = self.create_timer(1.0, self.publish_path)
        goal_ground = self.path_map[-1]
        goal_odom = self.final_path[-1]

        self.get_logger().info(f"PathPublisher inicializado a ground: {goal_ground} y odom: {goal_odom}")

    #============================================
    #Replanificación
    #============================================
    def generate_test_path(self):
        
        path_map = [(-7.0,         -7.0        ),
             ( 2.5,        -7.0        ),
             (-2.32962913, -5.70590477),
             ( 5.39777748, -3.63535241),
             ( 1.06765046, -1.13535241),
             (-2.93234954, -1.13535241),
             (-5.76077666,  1.69307471),
             (-6.79605285, -2.17062859),
             (-6.79605285, -6.17062859),
             (-1.29605285, -6.17062859),
             (-3.15954997,  0.78403736),
             ( 3.76865326,  4.78403736),
             ( 6.36672947,  3.28403736),
             ( 2.03660245,  5.78403736)]
        path_odom = map_path_to_odom(path_map, self.robot_map_pose)
        final_path = np.array(path_odom)
        return path_map, final_path
        
    def generate_rrt_path(self, start_point):
        
        path_map, final_path = compute_rrt_path(
            start_point,
            self.goal_point,
            self.robot_map_pose
        )
        return path_map, final_path
        
    def plan_from_pose(self):
        
        xo, yo = self.current_pose  # tomada de odom / ground_truth
        x0, y0, yaw0 = self.robot_map_pose
        xm = np.cos(yaw0)*xo - np.sin(yaw0)*yo
        ym = np.sin(yaw0)*xo + np.cos(yaw0)*yo

            # Traslación
        xm += x0
        ym += y0
        
        start = (xm, ym)
        
        self.get_logger().info(f"Replanning from {start}")

        self.path_map, self.final_path = self.generate_rrt_path(start)

        

    def replan_cb(self, msg):
        if not msg.data:
            return
        self.plan_from_pose()
        
        
        
    #============================================


       
    def odom_callback(self, msg: Odometry):

        x0 = msg.pose.pose.position.x
        y0 = msg.pose.pose.position.y

        self.current_pose = (x0, y0)
        
    def publish_path(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        

        for (x, y) in self.final_path:
            pose = PoseStamped()
            pose.header = path.header
            pose.header.stamp = path.header.stamp

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Orientación neutra (NO usamos yaw aquí)
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.pub_path.publish(path)
        

        #self.get_logger().info(f"Path publicado con {len(path.poses)} puntos")


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


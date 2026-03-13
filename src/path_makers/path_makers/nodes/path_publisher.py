import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Bool
import numpy as np

from path_makers.planning.rrt_exp import compute_rrt_path


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

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

        
        # ===== Waypoints definidos manualmente =====
        self.robot_map_pose=(-7, -7, np.pi/4)
        start_point = (-7, -7)
        self.goal_point = (6, 4)
        yawm = 0
        
        self.path_map, self.final_path = compute_rrt_path(
            start_point,
            yawm,
            self.goal_point,
            self.robot_map_pose
        )
       
        # Publicar periodicamente
        self.timer = self.create_timer(1.0, self.publish_path)
        goal_ground = self.path_map[-1]
        goal_odom = self.final_path[-1]

        self.get_logger().info(f"PathPublisher inicializado a ground: {goal_ground} y odom: {goal_odom}")
    #============================================
    #Replanificación
    #============================================
    def plan_from_pose(self):
        
        xo, yo, yawo = self.current_pose  # tomada de odom
        x0, y0, yaw0 = self.robot_map_pose
        xm = np.cos(yaw0)*xo - np.sin(yaw0)*yo
        ym = np.sin(yaw0)*xo + np.cos(yaw0)*yo

            # Traslación
        xm += x0
        ym += y0
        yawm = yaw0 + yawo
        
        start = (xm, ym)
        
        self.get_logger().info(f"Replanning from {start}")

        self.path_map, self.final_path = compute_rrt_path(
            start,
            yawm,
            self.goal_point,
            self.robot_map_pose
        )

        

    def replan_cb(self, msg):
        if not msg.data:
            return
        self.plan_from_pose()
        
        
        
    #============================================


       
    def odom_callback(self, msg: Odometry):

        x0 = msg.pose.pose.position.x
        y0 = msg.pose.pose.position.y
        yaw0 = self.get_yaw(msg.pose.pose.orientation)

        self.current_pose = (x0, y0, yaw0)
        
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
        
    def get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


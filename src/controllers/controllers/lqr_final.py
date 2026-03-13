#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import math
import numpy as np

from std_msgs.msg import Bool
from std_msgs.msg import Float64

import csv
import os


class SimpleGoalController(Node):
    def __init__(self):
        super().__init__('simple_goal_controller')
        
         # ===== PATH =====
        self.path_received = False
        self.path_points = []
        self.segments = []
        self.current_segment = 0
        self.current_pose = None
        self.replan_events = -1
        
        # ====== ESCAPE ============
        self.escape_start_time = 0.0
        self.escape = False
        self.escape_state = 0
        self.escape_start_time = None
        self.escape_active = False
        
        # ===== LIMITES =====
        self.v = 0.33
        self.w_max = np.deg2rad(90)
        
        # ===== GANANCIAS =====
        self.K = np.array([[2.22144147, 1.94942709]]) # velocidad ctte = 0.3, w_max = 90 grad/seg
        
        # ====== LQR =========
        self.J_demanded = 0.0      # costo usando u_raw
        self.J_applied = 0.0      # costo usando u_sat (w)
        self.J_states = 0.0       # costo sólo de los estados
        
        self.A = np.array([
            [0.0, self.v],
            [0.0, 0.0]
        ])

        self.B = np.array([
            [0.0],
            [1.0]
        ])

        # Matrices de costo
        self.Q = np.array([
            [2.0, 0.0],
            [0.0, 1.0]
        ])
        r = 1/(self.w_max**2)
        self.R = np.array([[r]])
        
        self.initial_time = None
        
        self.E_sat_total = 0.0
        self.P_sat_prev = 0.0


        # ===== TOLERANCIAS =====
        self.s_tol = 0.05
        self.angle_tol = 0.02
        self.lat_tol = 0.01
        
        # Subscribers 
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.sub_path = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        self.enabled = True
        self.create_subscription(
            Bool,
            '/nav/enable_controller',
            self.enable_cb,
            10
        )
        
        self.escape_sub = self.create_subscription(
            Bool,
            '/nav/escape_request',
            self.escape_cb,
            10
        )
        
        # Publishers
        self.pub_cmd = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.escape_done_pub = self.create_publisher(
            Bool,
            '/nav/escape_done',
            10
        )
        
        self.pub_P_sat   = self.create_publisher(Float64, "/metrics/P_sat",  10)
        self.pub_dP_sat  = self.create_publisher(Float64, "/metrics/dP_sat",  10)
        self.pub_E_sat   = self.create_publisher(Float64, "/metrics/E_sat", 10)
        self.pub_w = self.create_publisher(Float64, "/metrics/w",  10)
        self.pub_e_lat = self.create_publisher(Float64, "/metrics/e_lat",  10)
        self.pub_e_theta_pub = self.create_publisher(Float64, "/metrics/e_theta_deg",  10)
        self.pub_j_d = self.create_publisher(Float64, "/metrics/J_demanded",  10)
        self.pub_j_a = self.create_publisher(Float64, "/metrics/J_applied",  10)
        self.pub_j_s = self.create_publisher(Float64, "/metrics/J_states",  10)

                
        
        self.get_logger().info(
            f"LQR inicializado"
        )
    
    #=====================================
    #Funciónes de escape
    def escape_cb(self, msg):
        
        if msg.data and self.escape_state == 0:
            self.escape_active = True
            self.escape_state = 1
            self.escape_done_pub.publish(Bool(data=False))
            self.escape_start_time = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
            f"Escape initialized"
            )
        
    def escape_routine(self):
        if self.current_pose is None:
            self.get_logger().warn("Odom not received yet")
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        cmd = Twist()
        ESCAPE_IDLE = 0
        ESCAPE_BACK = 1
        ESCAPE_TURN = 2
        ESCAPE_DONE = 3
        
        giro = 2.0 #segundos
        retroceso = 2.0 #segundos
        if self.escape_state == ESCAPE_BACK:
            if t - self.escape_start_time < retroceso:
                cmd.linear.x = -0.6
                cmd.angular.z = 0.0
            else:
                self.escape_state = ESCAPE_TURN
                self.escape_start_time = t

        elif self.escape_state == ESCAPE_TURN:
            if t - self.escape_start_time < giro:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6
            else:
                self.escape_state = ESCAPE_DONE

        elif self.escape_state == ESCAPE_DONE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            # reset
            self.escape_active = False
            self.escape_state = ESCAPE_IDLE

            # MUY IMPORTANTE
            self.escape = False
            self.escape_done_pub.publish(Bool(data=True))
            self.get_logger().info(
            f"Escaped"
            )

        self.pub_cmd.publish(cmd)
        
    
    #=====================================
        
    def path_callback(self, msg: Path):
        if self.path_received:
            return

        self.path_points = []

        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.path_points.append((x, y))

        # Construcción de segmentos
        self.segments = []
        for i in range(len(self.path_points) - 1):
            p0 = self.path_points[i]
            p1 = self.path_points[i + 1]
            self.segments.append((p0, p1))

        self.current_segment = 0
        self.path_received = True

        self.get_logger().info(
            f"Path recibido con {len(self.segments)} segmentos"
        )
        self.get_logger().info("Avanzando")
        self.replan_events += 1
        #Reiniciar estado
        self.escape_done_pub.publish(Bool(data=False))
    
    #=====================================
    # FUNCIONES DE STOP
    #=====================================
    def enable_cb(self, msg):
        self.enabled = msg.data

    def publish_zero_cmd(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)
    #=====================================
    
    # =======================================================
    # ODOM CALLBACK A DEPURAR
    # =======================================================
    def odom_callback(self, msg: Odometry):
        if not self.path_received:
            return
        if not self.enabled:
            self.publish_zero_cmd()
            self.path_received = False
            return
        #Disparar escape
        if self.escape_active:
            self.escape_routine()
            #self.path_received = False
            return
        
        # Iniciar reloj para el tiempo:
        t = self.get_clock().now().nanoseconds * 1e-9
        
        if self.initial_time is None:
            self.initial_time = t
            dt = 0
        else:
            dt = t - self.initial_time
            self.initial_time = t
            
        self.current_pose = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        self.get_yaw(msg.pose.pose.orientation)
        )

        # ===============================
        # 1. ESTADO ACTUAL
        # ===============================
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw(msg.pose.pose.orientation)
        

        # ===============================
        # 2. SELECCIÓN DE WAYPOINT ACTIVO
        # ===============================
        if self.current_segment >= len(self.segments):
            # Trayectoria terminada
            self.pub_cmd.publish(Twist())
            
            #Conversión a ground truth para validación
            x0, y0, yaw0 = (-7, -7, np.pi/4)
            xm = np.cos(yaw0)*x - np.sin(yaw0)*y
            ym = np.sin(yaw0)*x + np.cos(yaw0)*y
                # Traslación
            xm += x0
            ym += y0
            
            metrics = self.add_metrics()
            self.print_metrics(metrics)
            
            # ---- CIERRE LIMPIO ----
            self.get_logger().info("Finalizando nodo...")
            self.destroy_node()
            rclpy.shutdown()
            return
            
        #====== CREACIÓN DE SEGMENTOS =======
        p0, p1 = self.segments[self.current_segment]
        self.x_g, self.y_g = p1
        
        seg_vec = np.array(p1) - np.array(p0)
        seg_len = np.linalg.norm(seg_vec)
        seg_dir = seg_vec / seg_len

        robot_pos = np.array([x, y])
        robot_vec = robot_pos - np.array(p0)
        s = np.dot(robot_vec, seg_dir)
        
        # ===============================
        # Saturación de proyección
        # ===============================
        s_clamped = np.clip(s, 0.0, seg_len)
        closest_point = np.array(p0) + s_clamped * seg_dir
        
        
        #Estados
        e_lat, e_theta_path = self.compute_errors(robot_pos, seg_dir, closest_point, yaw)
        
        
        cmd = Twist()
        
        # Segmento completado
        if (s >= seg_len - self.s_tol):
            self.current_segment += 1
            return
            
        # ========== LQR ====================
        x_states = np.array([[e_lat],   # e_y
              [e_theta_path]])  # e_theta
              
        u_raw = -self.K @ x_states
        w = np.clip(u_raw.item(), -self.w_max, self.w_max)
        
        
        # ========== Función costo ==================
        i_demanded_cost, i_applied_cost, i_states = self.update_cost(x_states, u_raw, w, dt)
        self.J_demanded += i_demanded_cost
        self.J_applied += i_applied_cost
        self.J_states += i_states
        
        
        #============ Métricas de saturación ==========
        # Error por saturación
        e_sat = u_raw - w

        # Potencia instantánea de saturación
        P_sat = e_sat**2

        # Estrés acumulado de saturación (área bajo la curva)
        self.E_sat_total += P_sat * dt
        
        # Pendiente del estrés para trigger
        if dt > 0.0:
            dP_sat = (P_sat - self.P_sat_prev) / dt
        else:
            dP_sat = 0.0

        self.P_sat_prev = P_sat
        
        # --- CAST EXPLÍCITO (CRÍTICO) ---
        P_sat_pub   = float(P_sat)
        dP_sat_pub  = float(dP_sat)
        E_sat_pub   = float(self.E_sat_total)
        w_pub = float(w)
        e_lat_pub = abs(float(e_lat))
        e_theta_pub = np.rad2deg(abs(float(e_theta_path)))
        j_d_pub = float(self.J_demanded)
        j_a_pub = float(self.J_applied)
        j_s_pub = float(self.J_states)

        # --- Publicación ROS2 ---
        self.pub_P_sat.publish(Float64(data=P_sat_pub))
        self.pub_dP_sat.publish(Float64(data=dP_sat_pub))
        self.pub_E_sat.publish(Float64(data=E_sat_pub))
        self.pub_e_lat.publish(Float64(data=e_lat_pub))
        self.pub_e_theta_pub.publish(Float64(data=e_theta_pub))
        
        self.pub_j_d.publish(Float64(data=j_d_pub))
        self.pub_j_a.publish(Float64(data=j_a_pub))
        self.pub_j_s.publish(Float64(data=j_s_pub))
        
        # ======= Mover robot ===========
        cmd.linear.x = self.v
        cmd.angular.z = w
        
        # ===============================
        # 7. PUBLICAR COMANDO
        # ===============================
        self.pub_cmd.publish(cmd)
    
    def update_cost(self, x_states, u_raw, w, dt):
        Q = self.Q
        R = self.R
        u_raw = np.array([u_raw])
        w = np.array([w])
        states_cost = x_states.T @ Q @ x_states
        # costo instantáneo ideal (sin saturación)
        f_raw = x_states.T @ Q @ x_states + u_raw.T @ R @ u_raw

        # costo instantáneo real (con saturación)
        f_sat = x_states.T @ Q @ x_states + w @ R @ w
        
        j_raw = f_raw * dt
        j_sat = f_sat * dt
        j_states = states_cost * dt

        return j_raw.item(), j_sat.item(), j_states.item()
    
    #Computar estados
    def compute_errors(self, robot_pos, seg_dir, closest_point, yaw):
    
        e_lat = np.cross(seg_dir, robot_pos - closest_point)

        theta_path = math.atan2(seg_dir[1], seg_dir[0])
        e_theta_path = self.normalize_angle(yaw - theta_path)
        
        return e_lat, e_theta_path
        

    # ===== UTILS =====

    def get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def add_metrics(self):
             
        metrics = {}
        
        metrics["Costo_aplicado"] = self.J_applied
        metrics["Costo_demandado"] = self.J_demanded
        metrics["Costo_estados"] = self.J_states
        
        degradation = 100 * (self.J_demanded - self.J_applied)/self.J_demanded
        metrics["Porcentaje_de_degradación"] = degradation
        metrics["Área_bajo_la_curva_saturación"] = self.E_sat_total.item()
        metrics["Replan events"] = self.replan_events
        
        return metrics
        
        
    def print_metrics(self, metrics):
        self.get_logger().info("===== MÉTRICAS =====")
        for k, v in metrics.items():
            self.get_logger().info(f"{k}: {v}")
        
            # Guardar en CSV
        self.save_metrics_csv(metrics)
            
    def save_metrics_csv(self, metrics, filename="metrics_tracker.csv"):
        self.get_logger().info("Guardando métricas en csv")
        file_exists = os.path.isfile(filename)

        with open(filename, "a", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=metrics.keys())

            # Escribe el header SOLO si el archivo no existía
            if not file_exists:
                writer.writeheader()

            writer.writerow(metrics)
        
def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoalController()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado. Cerrando nodo...")

    except RuntimeError as e:
        if "Context must be initialized" in str(e):
            print("[INFO] Nodo finalizado")
        else:
            raise e  # errores reales NO se ocultan
    finally:
        print('Ejecución Finalizada')


if __name__ == '__main__':
    main()

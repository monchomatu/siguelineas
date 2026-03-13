#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import math
import numpy as np
#Librerías para publicar con teclado
from std_msgs.msg import Float64, Bool
import sys
import termios
import tty
import csv
import os


class Supervisor(Node):

    def __init__(self):
        super().__init__('supervisor')
        # ===== LOGGERS ======
        self.time_log = []
        self.dist_log = []
        self.x_log = []
        self.y_log = []
        self.lateral_errors = []
        self.angular_errors = []
        self.e_theta_current = []
        self.tol_reached = False
        self.settle_times = []
        self.settle_t = None
        

       # ===== PATH =====
        self.path_received = False
        self.replan_requested = False
        self.retake_path = True
        self.path_points = []
        self.segments = []
        self.total_segments = []
        self.current_segment = 0
        
        self.out_event_active = False
        self.out_start_time = None

        self.total_out_time = 0.0
        self.out_events = 0
        self.out_log = []
        
        # ===== LQR =======
        self.umbral_dp = 3.277 # con replan
        #self.umbral_dp = 15000 # sin replan
        self.replan_active = False   # ← latch

        
        
        #==== LIDAR =======
        self.obstacle_active = False
        self.escape_routine_finished = False

        # ===== TOLERANCIAS =====
        self.s_tol = 0.1
        self.angle_tol = 0.2
        self.lat_tol = 0.1

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
        
        self.obstacle_sub = self.create_subscription(
            Bool,
            '/nav/obstacle_detected',
            self.obstacle_callback,
            10
        )
        
        self.escape_done_sub = self.create_subscription(
            Bool,
            '/nav/escape_done',
            self.escape_done_cb,
            10
        )
        
        self.sub_dp_sat  = self.create_subscription(
            Float64, 
            "/metrics/dP_sat", 
            self.dp_sat_cb,  
            10
        )

        #Publishers
        self.ctrl_pub = self.create_publisher(
            Bool,
            '/nav/enable_controller',
            10
        )

        self.replan_pub = self.create_publisher(
            Bool,
            '/nav/replan_request',
            10
        )
        
        self.escape_pub = self.create_publisher(
            Bool,
            '/nav/escape_request',
            10
        )


        self.get_logger().info("Supervisor listo")


    # FUNCIONES DE ESCAPE
    def escape_done_cb(self, msg):
        self.escape_routine_finished = msg.data
        if self.escape_routine_finished:    
            self.escape_pub.publish(Bool(data=False))
            self.ctrl_pub.publish(Bool(data=False))
            self.replan_pub.publish(Bool(data=True))
            
            self.get_logger().info(f"Obstáculo evadido: replanificando")
            self.obstacle_active = False
            self.replan_requested = True
            self.path_received = False
            self.retake_path = False
            
    # Función de replan por umbral
    def dp_sat_cb(self, msg):
        if self.replan_active:
            return  # ya disparó, no repetir

        if msg.data >= self.umbral_dp:
            self.replan_active = True
            self.get_logger().info(
                f"Umbral {self.umbral_dp} superado (dP_sat={msg.data:.2f}), replanificando"
            )

            self.ctrl_pub.publish(Bool(data=False))
            self.replan_pub.publish(Bool(data=True))
            self.replan_requested = True # para contar los segmentos totales
            self.path_received = False # para bloquear odometría
            self.retake_path = False # bloquear odometría
    
    def obstacle_callback(self, msg):
        if msg.data and not self.obstacle_active:
            if self.escape_routine_finished == False:
                self.obstacle_active = True
                self.escape_pub.publish(Bool(data=True))
                
                self.get_logger().info(f"Obstáculo detectado: iniciando maniobras de evasión")
                self.replan_requested = True # para contar los segmentos totales
                self.path_received = False # para bloquear odometría
                self.retake_path = False # bloquear odometría
        
            
    #==========================================================
    def path_callback(self, msg: Path):
        if self.path_received:
            return
        if self.obstacle_active:
            self.get_logger().info(f"Path cb, escape acabado: {self.escape_routine_finished}, obstáculo activo: {self.obstacle_active}")
            return
        
        self.path_points = []
        

        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.path_points.append((x, y))
            
        #-------------------------------------------------
        start_point = self.path_points[0]
        current = self.current_segment
        #Guardar los segmentos sólo hasta el último recorrido e interrumpido en el punto start_point
        if self.replan_requested:
            for m in range(current-1):
                self.total_segments.append(self.segments[m])
            self.total_segments.append((self.segments[current][0], start_point))
            self.replan_requested = False
        #------------------------------------------------
        
        # Construcción de segmentos
        self.segments = []
        for i in range(len(self.path_points) - 1):
            p0 = self.path_points[i]
            p1 = self.path_points[i + 1]
            self.segments.append((p0, p1))
        
        
        self.current_segment = 0
        self.path_received = True
        self.replan_requested = False

        self.get_logger().info(
            f"Path recibido, iniciando marcha"
        )
        
        #Reanudar marcha
        self.ctrl_pub.publish(Bool(data=True))
        self.retake_path = True
        self.obstacle_active = False
        self.replan_active = False

    def odom_callback(self, msg: Odometry):
        if not self.path_received:
            return
        #No iniciar el reloj hasta volver a avanzar
        if self.retake_path == False:
            return
            
        # Iniciar reloj para el tiempo:
        t = self.get_clock().now().nanoseconds * 1e-9

        
        #ESTADO ACTUAL
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw(msg.pose.pose.orientation)
        
        #Delegar
        self.step_supervisor(x, y, yaw, t)

    
    # =================
    # SUPERVISOR
    # =================
    def step_supervisor(self, x, y, yaw, t):
        
        # SELECCIÓN DE WAYPOINT ACTIVO
        if self.current_segment >= len(self.segments):
            # Trayectoria terminada
            self.get_logger().info("Trayectoria completada")
            
            #===== PUBLICAR MÉTRICAS =====
            metrics = self.compute_metrics()
            self.print_metrics(metrics)
            
            # ---- CIERRE LIMPIO ----
            self.get_logger().info("Finalizando nodo...")
            self.destroy_node()
            rclpy.shutdown()
            return
            
        # CREACIÓN DE SEGMENTOS
        p0, p1 = self.segments[self.current_segment]
        self.x_g, self.y_g = p1
        
        # diferencial de distancia recorrida
        dx = self.x_g - x
        dy = self.y_g - y
        d = math.sqrt(dx**2 + dy**2)
        
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
        
        
        # ERROR LATERAL Y ANGULAR
        e_lat, e_theta_path = self.compute_errors(robot_pos, seg_dir, closest_point, yaw)
        
        # LOG PARA MÉTRICAS
        #self.get_logger().info(f"segment={self.current_segment}, d={(seg_len - s):.2f}, e_theta={e_theta_path:.2f}, e_lat:{e_lat:.2f}")
        #============================================
        
        # ===============================
        # MÉTRICA DE RECUPERACIÓN LATERAL
        # ===============================

        # --- Entrada a desviación ---
        if not self.out_event_active and abs(e_lat) > self.lat_tol:
            self.out_event_active = True
            self.out_start_time = t
            self.out_events += 1

        # --- Salida de desviación ---
        elif self.out_event_active and abs(e_lat) < (0.5 * self.lat_tol):
            out_dt = t - self.out_start_time
            self.total_out_time += out_dt
            self.out_log.append(out_dt)
            self.out_event_active = False
            self.out_start_time = None

        # ===============================
        # Observación del tracker
        # ===============================
                
        if (s >= seg_len - self.s_tol):                       
            # FINAL ALCANZADO
            # GUARDAR Y REINICIAR VALORES DE SEGMENTO
            self.current_segment += 1
            #Agregar los nuevos errores laterales por segmento
            self.e_theta_current = []
            self.settle_t = None
            self.tol_reached = False
    
        
        # GUARDAR VALORES HISTÓRICOS
        self.e_theta_current.append(e_theta_path)
        self.lateral_errors.append(e_lat)
        self.angular_errors.append(abs(e_theta_path))
        self.time_log.append(t)
        self.dist_log.append(d)
        self.x_log.append(x)
        self.y_log.append(y)
    
    
    # ===== UTILS =====

    def get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
        
        
    # ===== Calcular error lateral y angular =====
    
    def compute_errors(self, robot_pos, seg_dir, closest_point, yaw):
    
        e_lat = np.cross(seg_dir, robot_pos - closest_point)

        theta_path = math.atan2(seg_dir[1], seg_dir[0])
        e_theta_path = self.normalize_angle(yaw - theta_path)
        
        return e_lat, e_theta_path
    
    def compute_metrics(self):
        
        metrics = {}
        
        # ===============================
        # TIEMPO TOTAL DE CONVERGENCIA
        # ===============================
        total_time = self.time_log[-1] - self.time_log[0]
        metrics["Total_time"] = total_time

        # ===============================
        # LONGITUD DE TRAYECTORIAS
        # ===============================
        # Longitud planeada
        planned_length = 0.0
        if len(self.total_segments) == 0:
            for j in range(1, len(self.path_points)):
                pdx = self.path_points[j][0] - self.path_points[j - 1][0]
                pdy = self.path_points[j][1] - self.path_points[j - 1][1]
                planned_length += math.sqrt(pdx * pdx + pdy * pdy)
            metrics["Path_length no replan"] = planned_length
        else:
            #recuperar los primeros puntos de todo el total_lentgh
            total_points = []
            for n in range(len(self.total_segments)):
                punto = self.total_segments[n][0]
                total_points.append(punto)
            punto = self.total_segments[-1][1]
            total_points.append(punto)
            
            for q in range(1, len(total_points)):
                pdx = total_points[q][0] - total_points[q - 1][0]
                pdy = total_points[q][1] - total_points[q - 1][1]
                planned_length += math.sqrt(pdx * pdx + pdy * pdy)
                
            for j in range(1, len(self.path_points)):
                pdx = self.path_points[j][0] - self.path_points[j - 1][0]
                pdy = self.path_points[j][1] - self.path_points[j - 1][1]
                planned_length += math.sqrt(pdx * pdx + pdy * pdy)
                
            metrics["Path_length with replan"] = planned_length
        
        # LONGITUD REAL
        length = 0.0
        for i in range(1, len(self.x_log)):
            dx = self.x_log[i] - self.x_log[i - 1]
            dy = self.y_log[i] - self.y_log[i - 1]
            length += math.sqrt(dx * dx + dy * dy)

        metrics["Real_length"] = length
        
        # Relación entre distancia real y planificada
        if planned_length > 0.0:
            ratio = 100 * length / planned_length
        else:
            ratio = None
        metrics["Real_vs_planned_length"] = ratio
        # ===============================
        # RMSE LATERAL
        # ===============================
        metrics["RMSE_lateral"] = np.sqrt(np.mean(np.square(self.lateral_errors)))
        # ===============================
        # RMSE ANGULAR
        # ===============================
        ang = np.array(self.angular_errors)
        metrics["RMSE_angular"] = np.sqrt(np.mean(ang ** 2))
        #Settle times mean
        if self.out_events > 0:
            avg_settle_time = self.total_out_time / self.out_events
        else:
            avg_settle_time = 0.0
        metrics["avg_settle_time"] = np.mean(avg_settle_time)
        metrics["Total_deviation_events"] = self.out_events
        metrics["Max_deviation_time"] = max(self.out_log)
        metrics["Time_deviated_percentage"] = 100 * self.total_out_time/total_time
        
        # ERROR ANGULAR MÁXIMO
        metrics["max_angular_error"] = np.max(np.abs(ang))
        # ERROR LATERAL MÁXIMO
        metrics["max_lateral_error"] =max(np.abs(self.lateral_errors))
        

        return metrics
        
    def print_metrics(self, metrics):
        self.get_logger().info("===== MÉTRICAS =====")
        for k, v in metrics.items():
            self.get_logger().info(f"{k}: {v}")
        
            # Guardar en CSV
        self.save_metrics_csv(metrics)
            
    def save_metrics_csv(self, metrics, filename="metrics_supervisor.csv"):
        file_exists = os.path.isfile(filename)

        with open(filename, "a", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=metrics.keys())

            # Escribe el header SOLO si el archivo no existía
            if not file_exists:
                writer.writeheader()

            writer.writerow(metrics)


def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()
    
    try:
        rclpy.spin(node)    
    finally:
        print('Ejecución Finalizada')
    try:
        node.destroy_node()
        rclpy.shutdown()
        
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado. Cerrando nodo...")

    except RuntimeError as e:
        if "Context must be initialized" in str(e):
            print("[INFO] Nodo finalizado")
        else:
            raise e  # ❗ errores reales NO se ocultan
    finally:
        print('Nodo Cerrado')

if __name__ == '__main__':
    main()


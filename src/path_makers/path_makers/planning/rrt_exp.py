import numpy as np
from path_makers.planning.Bit_Map import BitmapMap
from ament_index_python.packages import get_package_share_directory
import os


def extract_path_from_edges(edges, start_point):
    """
    edges: list of (parent, child)
    start: (x,y)
    goal: (x,y)
    """
    path_edges = []

    # Comenzar desde el último edge (goal)
    current_edge = edges[-1]
    path_edges.append(current_edge)

    current_parent = current_edge[0]

    while current_parent != start_point:
        found = False
        for e in edges:
            if e[1] == current_parent:
                path_edges.append(e)
                current_parent = e[0]
                found = True
                break

        if not found:
            raise RuntimeError("No se pudo reconstruir el path")

    path_edges.reverse()
    path = [path_edges[0][0]]  # start
    for e in path_edges:
        path.append(e[1])
    return path

def map_path_to_odom(path_map, robot_map_pose):
    """
    path_map: list of (x,y) en frame mapa
    robot_map_pose: (x0, y0, yaw0) en mapa
    """
    x0, y0, yaw0 = robot_map_pose
    theta = -yaw0  # rotar

    path_odom = []

    for x, y in path_map:
        # Traslación
        xt = x - x0
        yt = y - y0

        # Rotación
        xo =  np.cos(theta)*xt - np.sin(theta)*yt
        yo =  np.sin(theta)*xt + np.cos(theta)*yt

        path_odom.append((xo, yo))

    return path_odom

def compute_rrt_path(start_point, yawm, goal_point, robot_map_pose):

    # Cargar mapa
    pkg_dir = get_package_share_directory('stage_utils')

    bitmap_path = os.path.join(
        pkg_dir,
        'world',
        'bitmaps',
        'cave.png'
    )

    bitmap = BitmapMap(bitmap_path, world_size=16.0)
    
    # Tamaño en stage del robot según documentación = 0.511
    bitmap.inflate(robot_radius=0.3)
    # Definir Starting Point
    goal_radius = 1.0
    near = (0,0)
    p = start_point
    p_tree = start_point
    nodes = []
    edges = []
    nodes.append(start_point)
    step_size = 0.5

    # Auxiliares
    SAMPLES = 500
    STEP_DELAY = 0.01
    
    # Nodo inicial orientado por yaw
    theta_max = np.deg2rad(50)
    first_node = None
    d = step_size

    for _ in range(20):  # intentos para evitar obstáculo frontal
        dtheta = np.random.uniform(-theta_max, theta_max)
        thetam = yawm + dtheta

        x1 = start_point[0] + d * np.cos(thetam)
        y1 = start_point[1] + d * np.sin(thetam)

        if not bitmap.is_occupied(x1, y1):
            if bitmap.collision_free(start_point, (x1, y1)):
                first_node = (x1, y1)
                break

    if first_node is not None:
        nodes.append(first_node)
        edges.append((start_point, first_node))

    # Sampling loop
    goal_reached = False
    while goal_reached == False:

        # Sample en coordenadas mundo
        x = np.random.uniform(-bitmap.half_size, bitmap.half_size)
        y = np.random.uniform(-bitmap.half_size, bitmap.half_size)
        

        # Consultar ocupación
        if bitmap.is_occupied(x, y):   # obstáculo
            color = None
        else:
                  
            p = (x,y)
            near = min(nodes, key=lambda n: np.hypot(p[0] - n[0], p[1] - n[1]))

            if bitmap.collision_free(near, p):
                direction = (p[0] - near[0], p[1] - near[1])
                dist = np.linalg.norm(direction)
                direction_unit = (direction[0]/dist,direction[1]/dist)
                

                if dist <= step_size:
                    p_tree = p
                else:
                    p_tree = (near[0] + step_size * direction_unit[0], near[1] + step_size * direction_unit[1])

                goal_dist = np.hypot(p_tree[0] - goal_point[0], p_tree[1] - goal_point[1])    

                if goal_dist <= goal_radius:
                    goal_reached = True
                    p_tree = goal_point
                
                
                
                nodes.append(p_tree)
                edges.append((near, p_tree))
                if goal_reached:
                    path = extract_path_from_edges(edges, start_point)
                    path = np.array(path)



    
    path_odom = map_path_to_odom(path, robot_map_pose)
    final_path = np.array(path_odom)
    return path, final_path





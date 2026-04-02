# Siguelíneas - Simulación con ROS2 y Stage

Este repositorio implementa un sistema de navegación reactiva basado en RRT para un robot tipo uniciclo en simulación usando Stage y ROS2.

---

## Requisitos

- ROS2 Jazzy
- Python 3
- colcon
- rosdep

---

## Instalación

Clonar el repositorio:

```bash
git clone https://github.com/monchomatu/siguelineas.git
cd siguelineas/src
```
Clonar dependencias:

```bash
git clone https://github.com/tuw-robotics/Stage.git
git clone https://github.com/tuw-robotics/stage_ros2.git
```
Volver a la raíz del workspace e instalar dependencias del sistema:
```bash
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

Compilar y cargar:
```bash
colcon build
source install/setup.bash
```

---
## Ejecutar simulación

Dar permisos al script:
```bash
chmod +x run_experiments.sh
```

El siguiente comando tiene la sintáxis para correr múltiples experimentos:
- archivo
- cantidad de carreras con replanificación
- ruido en actuadores

```bash 
./run_experiments.sh 1 1 false
```


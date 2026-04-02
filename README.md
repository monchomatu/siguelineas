# Siguelíneas - Simulación con ROS2 y Stage

Este repositorio implementa un sistema de navegación reactiva basado en RRT para un robot tipo uniciclo en simulación usando Stage y ROS2 Jazzy.

Ejecutar source install/setup.bash en cada nueva terminal

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
Clonar dependencias externas:

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
- cantidad de carreras sin replanificación
- ruido en actuadores (true/false)

```bash 
./run_experiments.sh 1 1 false
```
---
## Estructura del proyecto
- stage_utils: mundos, mapas y configuraciones RViz modificados del original stage_ros2
- path_makers: planificación de trayectorias y publicación de recorrido
- controllers: control y monitoreo de navegación del robot y launchers
- perception: procesamiento de sensores
- En cada ejecución se creará una carpeta llamada results con un csv de las métricas de la corrida y una imágen del recorrido
---

## Dependencias externas

Este trabajo se apoya en herramientas desarrolladas por TU Wien Robotics Group:

- Stage (simulador 2D de robots): https://github.com/tuw-robotics/Stage
- stage_ros2 (integración de Stage con ROS2): https://github.com/tuw-robotics/stage_ros2

Se agradece a sus respectivos autores por el desarrollo de estas herramientas.

---

## Licencia y créditos

Este proyecto hace uso de software de terceros. Cada dependencia mantiene su propia licencia, la cual debe ser respetada.

Para más información, consultar los repositorios originales.
---

## Autor
Ramón Herrera

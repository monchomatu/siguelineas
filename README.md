# Siguelíneas - Simulation with ROS2 and Stage

This repository implements a reactive navigation system based on RRT for a unicycle-type robot in simulation using Stage and ROS2 Jazzy.

Run `source install/setup.bash` in each new terminal

---

## Requirements
- ROS2 Jazzy
- Python 3
- colcon
- rosdep

---

## Installation

Clone the repository:
```bash
git clone https://github.com/monchomatu/siguelineas.git
cd siguelineas/src
```

Clone external dependencies:
```bash
git clone https://github.com/tuw-robotics/Stage.git
git clone https://github.com/tuw-robotics/stage_ros2.git
```

Go back to the workspace root and install system dependencies:
```bash
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

Build and load:
```bash
colcon build
source install/setup.bash
```

---

## Running the simulation

Grant permissions to the script:
```bash
chmod +x run_experiments.sh
```

The following command has the syntax to run multiple experiments:
- file
- number of runs with replanning
- number of runs without replanning
- actuator noise (true/false)

```bash
./run_experiments.sh 1 1 false
```

---

## Project structure
- **stage_utils**: worlds, maps, and RViz configurations modified from the original stage_ros2
- **path_makers**: trajectory planning and path publishing
- **controllers**: robot navigation control and monitoring, and launchers
- **perception**: sensor processing
- Each run will create a folder called `results` containing a CSV with the run metrics and an image of the trajectory

---

## External dependencies

This work builds upon tools developed by the TU Wien Robotics Group:
- **Stage** (2D robot simulator): https://github.com/tuw-robotics/Stage
- **stage_ros2** (Stage integration with ROS2): https://github.com/tuw-robotics/stage_ros2

Credits go to their respective authors for the development of these tools.

---

## License and credits

This project makes use of third-party software. Each dependency maintains its own license, which must be respected. For more information, refer to the original repositories.

---

## Author

Ramon Herrera

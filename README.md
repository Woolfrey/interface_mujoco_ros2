# MuJoCo-ROS2 Interface

This class establishes communication between a MuJoCo simulation and ROS2.

> [!WARNING]
> This repository is still under construction. <br>
> It is currently only publishing joint states. Work to receive joint commands is underway. <br>
> Stay tuned! 

## Features

- Launches a MuJoCo simulation.
- Creates a ROS2 node for communication.
- Publishes joint state information over ROS2.
- Provides real-time visualization of the robot and its environment.
- Allows manual interaction with the simulation.

## Dependencies

- ROS2 (Humble Hawksbill or later)
- MuJoCo
- GLFW
- Standard C++ libraries

## Installation

### Prerequisites

Ensure that you have ROS2 and MuJoCo installed on your system.

1. **Install ROS2:**
   Follow the [ROS2 installation guide](https://docs.ros.org/en/foxy/Installation.html) for your operating system.

2. **Install MuJoCo:**
   Download and install MuJoCo from the [official website](https://mujoco.org/).

3. **Install GLFW:**
```
   sudo apt-get install libglfw3-dev
```

### Building the Project

Clone the repository:
```
git clone https://github.com/Woolfrey/interface_mujoco_ros2
cd interface_mujoco_ros2
```
Build the package:
```
colcon build
```
Source the ROS2 workspace:
```
source install/setup.bash
```

### Usage

#### Launching the Interface

To launch the MuJoCo ROS2 Interface, run the following command:

```
ros2 run mujoco_interface mujoco_interface_node /path/to/your/robot_model.xml
```

Replace /path/to/your/robot_model.xml with the path to your MuJoCo XML model or scene file.

## Contributing

Contributions are welcome! Please fork the repository and submit pull requests.

1. Fork the Project
2. Create your Feature Branch (git checkout -b feature/AmazingFeature)
3. Commit your Changes (git commit -m 'Add some AmazingFeature')
4. Push to the Branch (git push origin feature/AmazingFeature)
5. Open a Pull Request

## License

Distributed under the GNU General Public License. See LICENSE for more information.
Contact

Jon Woolfrey - jonathan.woolfrey@gmail.com

Project Link: https://github.com/yourusername/MuJoCo-ROS2-Interface

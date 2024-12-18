<a name="top"></a>
# MuJoCo-ROS2 Interface

> [!NOTE]
> This project is a work-in-progress, so new features will be added slowly.
> Feel free to contribute your own improvements.

This class establishes communication between a MuJoCo simulation and ROS2.

It publishes a `sensor_msgs::msg::JointState` topic for you to use, and allows commands to the joints via a `std_msgs::msg::Float64MultiArray` topic:

<p align = "center">
<img src ="https://github.com/user-attachments/assets/fff5be63-dc23-4c33-97a6-83f376ffccc6" width = "800" height = "auto" />
</p>

It can run in `POSITION`, `VELOCITY`, or `TORQUE` mode which may be set via the config and/or launch file(s).

>[!TIP]
> You can download MuJoCo robot models [here](https://github.com/google-deepmind/mujoco_menagerie.git).

- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
   - [Prerequisites](#prerequisites)
   - [Building the Project](#building-the-project)
   - [Usage](#usage)
   - [Launching the Interface](#launching-the-interface)
- [Contributing](#contributing)
- [License](#license)

## Features

- Launches a MuJoCo simulation.
- Creates a ROS2 node for communication.
- Publishes joint state information over ROS2.
- Provides real-time visualization of the robot and its environment.
- Allows manual interaction with the simulation.

[⬆️ Back to top.](#top)

## Dependencies

- ROS2 (Humble Hawksbill or later)
- MuJoCo
- GLFW
- Standard C++ libraries

[⬆️ Back to top.](#top)

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

[⬆️ Back to top.](#top)

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

[⬆️ Back to top.](#top)

### Usage

#### Launching the Interface

There are 2 different control modes currently available:

To run **velocity control**, you can launch:
```
ros2 launch mujoco_interface velocity_mode.py
```
This requires that the topic `/joint_commands` contains an array of velocites (in rad/s).

To run **torque control**, you can launch:
```
ros2 launch mujoco_interface torque_mode.py
```
This requires that the topic `/joint_commands` contains an array of torques (Nm).

> [!TIP]
> In torque mode, gravity and Coriolis torques are automatically compensated for.

[⬆️ Back to top.](#top)

## Contributing

Contributions are welcome! Please fork the repository and submit pull requests.

1. Fork the Project
2. Create your Feature Branch (git checkout -b feature/AmazingFeature)
3. Commit your Changes (git commit -m 'Add some AmazingFeature')
4. Push to the Branch (git push origin feature/AmazingFeature)
5. Open a Pull Request

[⬆️ Back to top.](#top)

## License

Distributed under the GNU General Public License. See LICENSE for more information.
Contact

Jon Woolfrey - jonathan.woolfrey@gmail.com

[⬆️ Back to top.](#top)

# Position Controller
### _By Ricardo Navarro_


This repository contains the code for a position controller designed for an engine, utilizing an ESP32 microcontroller, Micro-ROS, and a PID controller. The system is aimed at controlling the position of the engine, ensuring precise and stable operation.

## Components Used
- ESP32: ESP32 is a series of low-cost, low-power system-on-chip microcontrollers with integrated Wi-Fi and dual-mode Bluetooth.
- Micro-ROS: Micro-ROS extends the capabilities of ROS 2 to microcontrollers, enabling communication between embedded systems and higher-level ROS 2 nodes.
- PID Controller: A PID (Proportional-Integral-Derivative) controller is a control loop feedback mechanism used in systems where precise control of the output is required.

## Functionality
The position controller system utilizes the ESP32 microcontroller to read sensor data from the engine and calculate the necessary control signals using a PID controller. Micro-ROS facilitates communication between the ESP32 and higher-level ROS 2 nodes, allowing for integration with other ROS 2-based systems

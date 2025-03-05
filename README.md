# Bucky-Arm

A 5-axis robot arm project with a Python-based control application.

## Overview

This project consists of:
1. An ESP32-controlled robot arm with 5 stepper motors
2. A web interface loaded on the ESP32 for basic control
3. A Python application for advanced control and visualization

## Components

- **Hardware**: Custom 3D printed parts with an ESP32 controller
- **Arduino Code**: Controls the stepper motors and hosts a web server
- **Web Interface**: Basic control interface served by the ESP32
- **Python Application**: Advanced control with 3D visualization

## Python Application Features

- IP configuration dialog for robot arm and camera connections
- Individual joint control with sliders and buttons
- Real-time 3D visualization of the robot arm
- Manipulable 3D model with mouse controls
- Camera feed display from ESP32-CAM
- Clean, intuitive user interface

## Requirements

- Python 3.6+
- PyQt5
- PyQtWebEngine
- NumPy
- PyQtGraph
- Requests

## Installation

```bash
pip install PyQt5 PyQtWebEngine numpy pyqtgraph requests
sudo apt install python3-qtpy
```

## Usage

1. Make sure your ESP32 robot arm is powered on and connected to your network
2. If using a camera, ensure the ESP32-CAM is also powered on and connected
3. Run the Python application:

```bash
python bucky_arm_controller.py
```

4. Enter the IP addresses when prompted
5. Use the interface to control the robot arm

## License

See the LICENSE file for details.
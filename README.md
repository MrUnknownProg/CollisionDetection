# Crane Accident Prevention System

## Overview
This project implements a collision avoidance system for construction cranes using Raspberry Pi. The module, placed at the tip of the crane, utilizes LIDAR, a webcam, ultrasonic & proximity sensors, and a GSM module to detect obstacles, predict motion, and prevent accidents.

## Features
- **LIDAR Mapping:** Scans the surroundings and detects foreign objects.
- **Webcam AI Processing:** Identifies nearby cranes and predicts their movement.
- **Ultrasonic & Proximity Sensors:** Acts as an additional precautionary measure.
- **GSM Module (A7670C):** Provides real-time GPS coordinates.
- **Buzzer Alert System:** Increases intensity as objects get closer.
- **Crane Control:** Slows down or stops the crane based on detected threats.

## Hardware Requirements
- Raspberry Pi 4B
- RPLIDAR (e.g., RPLIDAR A1/A2)
- Webcam
- Ultrasonic Sensor (HC-SR04)
- Proximity Sensor
- A7670C GSM Module
- Buzzer
- Relay Module (for controlling crane motor)
- GPIO Wiring & Power Supply

## Installation
1. Clone the repository:
   ```sh
   git clone https://github.com/your-repo/crane-safety.git
   cd crane-safety
   ```
2. Install dependencies:
   ```sh
   pip install opencv-python numpy RPi.GPIO adafruit-circuitpython-rplidar adafruit-circuitpython-ads1x15 geopy ultralytics
   ```
3. Connect all hardware components properly.
4. Run the script:
   ```sh
   python crane_safety.py
   ```
   
## How It Works
- The LIDAR continuously scans the area, detecting objects within a predefined range.
- The webcam uses AI to identify other cranes and predict their movement.
- If an object is detected within a critical range, the system slows down or stops the crane.
- The buzzer increases its intensity as an object gets closer.
- GPS coordinates are fetched using the GSM module.
- The crane is halted if an obstacle is detected too close or if another crane is approaching.

## Configuration
- Modify `BUZZER_HIGH_INTENSITY` and `BUZZER_LOW_INTENSITY` in the script to set dynamic alert distances.

## Safety Precautions
- Ensure all sensors are properly calibrated.
- Regularly test the system before use.
- Implement additional manual override options if needed.


For any issues, please raise a ticket in the repository or contact the development team.


# Roam-E 🤖
## WRO - Team Paragon
### A small intelligent robot car that can roam around autonomously by detecting traffic signals

---

## 📹 Project Demo Video

[![Roam-E Demo Video](https://img.youtube.com/vi/jhrejO-gC3s/maxresdefault.jpg)](https://www.youtube.com/watch?v=jhrejO-gC3s)

*Click the image above to watch our robot in action!*

---

## 🚗 Vehicle Overview

Roam-E is an autonomous robot car designed for the World Robot Olympiad (WRO) competition. The vehicle combines advanced computer vision, sensors, and intelligent control algorithms to navigate autonomously while detecting and responding to traffic signals and obstacles.

### Vehicle Images

| Top View | Right Side View |
|----------|-----------------|
| ![Top View](Team%20&%20Vehicle%20Photos/Vehicle%20Photos/top.jpg) | ![Right View](Team%20&%20Vehicle%20Photos/Vehicle%20Photos/right.jpg) |

| Front View | Back View | Left View | Bottom View |
|------------|-----------|-----------|-------------|
| ![Front](Team%20&%20Vehicle%20Photos/Vehicle%20Photos/front.jpg) | ![Back](Team%20&%20Vehicle%20Photos/Vehicle%20Photos/back.jpg) | ![Left](Team%20&%20Vehicle%20Photos/Vehicle%20Photos/left.jpg) | ![Bottom](Team%20&%20Vehicle%20Photos/Vehicle%20Photos/bottom.jpg) |

---

## 🏗️ System Architecture

### Hardware Components

#### **Processors**
- **Jetson Nano**: Handles computer vision processing, image analysis, and AI-based decision making
- **Arduino Uno**: Controls physical components (motors, servos, sensors) and acts as communication bridge

#### **Sensors**
- **Raspberry Pi Camera Module 3**: Enhanced with 0.45x wide-angle lens for broader field of view
- **HC-SR04 Ultrasonic Distance Sensors**: Three sensors (left, right, center) for obstacle detection
- **10-DOF IMU Sensor (MPU9255 + BMP280)**: Provides precise motion and orientation measurements (0-1080°)

#### **Actuators**
- **MG-996 360° Servo Motor**: Front wheel steering with high torque and precision
- **25GA370 300RPM Metal Gear DC Motor**: Rear wheel drive (upgraded from JGA25-370 for better performance)
- **BTS7960 Motor Drive Module**: Motor control and power management

---

## ⚡ Power & Sense Management

### Power System
- **Primary Power**: 1800mAh LiPo Battery (11.8V × 1.8A = 21.24W)
- **Voltage Boosting**: LM2577 DC-DC Step-up Converter (13V output for DC motor)
- **Voltage Regulation**: LM2596 Step-down Converter (5.1V for Jetson Nano, servo, Arduino)
- **Power Distribution**: Efficient power routing to all components

### Circuit Design
![Power Circuit](Power%20&%20Sense%20Management/paragon_circuit.png)

### Sensor Specifications
- **Distance Measurement**: HC-SR04 sensors with 250cm maximum range
- **Vision Processing**: 272×204 resolution at 90fps capture rate
- **Motion Tracking**: 10-DOF IMU with accelerometer, gyroscope, and magnetometer
- **Angle Detection**: 0-1080° range with automatic stop at 1050°

---

## 🚙 Mobility Management

### Chassis & Components
- **Base**: [4WD Smart Car Robot Chassis](https://www.elecrow.com/4wd-smart-car-robot-chassis-for-arduino-servo-steering.html)
- **Wheels**: 65mm diameter, 54mm width for enhanced maneuverability
- **Weight Optimization**: Thin wheels reduce overall weight and improve efficiency

### Motor Control
- **Front Steering**: MG-996 servo centered at 40° (0° = left turn, 110° = right turn)
- **Rear Drive**: 25GA370 motor with BTS7960 driver for forward/reverse motion
- **Speed Control**: PWM-based speed regulation with directional control

### Assembly Features
- Three-layer chassis design (metal base + dual acrylic layers)
- Integrated sensor mounting points
- Cable management system with ties and heat shrink tubing
- Anti-collision protection

---

## 🧠 Obstacle Management & AI

### Computer Vision Pipeline
The robot uses advanced image processing for autonomous navigation:

#### **Image Processing**
- **Color Detection**: HSV-based detection for red/green traffic signals, orange/blue obstacles
- **Wall Detection**: Black line following using morphological operations
- **Blob Analysis**: Contour detection with area-based filtering (>100px threshold)

#### **Decision Making Algorithm**
```python
# Simplified decision flow
1. Capture frame from dual CSI cameras
2. Split image into color masks (red, green, orange, blue, black)
3. Detect walls using left/right ROI analysis
4. Calculate PID-based wall following
5. Detect traffic signals (red = left turn, green = right turn)
6. Combine wall-following + traffic signal decisions
7. Send motor commands via serial communication
```

#### **Control System**
- **PID Controller**: Kp=3000, Ki=0, Kd=0 for wall following
- **Turn Mapping**: [-1000, 1000] → [0, 110] servo range
- **Real-time Processing**: Continuous frame analysis at ~10fps

### Debug Interface
- **Web Monitoring**: Flask-based debug server on port 5000
- **Live Visualization**: Real-time frame display with detection overlays
- **Parameter Monitoring**: Turn values, detection areas, and decision metrics

---

## 💻 Software Architecture

### Challenge Round (Python)
**File**: `Source Code/Challenge Round/test_v8.py`

**Key Classes**:
- `CameraHandler`: Dual CSI camera management
- `ImageProcessor`: HSV color detection and blob analysis
- `WallAnalyzer`: PID-based wall following
- `TurnCalculator`: Traffic signal decision making
- `CarController`: Serial communication with Arduino
- `DebugServer`: Web-based monitoring interface

### Open Round (Arduino)
**File**: `Source Code/Open Round/qualifier.ino`

**Features**:
- Ultrasonic sensor-based navigation
- PID control for wall following
- Automatic lap counting (12 laps)
- Direction detection and path optimization

### Final Round Variants
- **`final_round.ino`**: IMU-based navigation with angle tracking
- **`final_round_wa.ino`**: Wall-avoidance with U-turn capability

---

## 🛠️ Setup & Installation

### Hardware Setup
1. **Assemble Chassis**: Follow the 4WD chassis assembly instructions
2. **Mount Components**:
   - Jetson Nano and Arduino on middle layer
   - LiPo battery in center compartment
   - Sensors on front/sides of chassis
3. **Wiring**: Connect according to the circuit diagram
4. **Calibration**: Set servo center position and sensor offsets

### Software Installation

#### Prerequisites
```bash
# For Jetson Nano
sudo apt update
sudo apt install python3-pip python3-opencv
pip3 install jetcam flask pyserial numpy

# For Arduino
# Install Arduino IDE and required libraries:
# - Servo.h
# - NewPing.h
# - MPU6050_light.h
# - PID_v1.h
```

#### Running the System

**Challenge Round (Computer Vision)**:
```bash
cd "Source Code/Challenge Round"
python3 test_v8.py
# Access debug interface at http://[jetson-ip]:5000
```

**Open Round (Sensor-based)**:
```bash
# Upload qualifier.ino to Arduino
# Press button to start autonomous navigation
```

### Configuration
- **Camera Settings**: 272×204 resolution, 90fps
- **Serial Communication**: 9600 baud rate
- **Servo Range**: 0-110° (40° center)
- **Motor Speed**: 60-120 PWM range
- **IMU Stop Angle**: 1050° (approximately 3 full rotations)

---

## 🎯 Competition Modes

### Open Round
- **Objective**: Navigate around track using ultrasonic sensors
- **Strategy**: PID-based wall following with automatic direction detection
- **Completion**: 12 laps with timing optimization

### Challenge Round
- **Objective**: Navigate while responding to traffic signals
- **Strategy**: Computer vision-based obstacle detection and traffic signal recognition
- **Features**: Real-time decision making, web-based debugging

---

## 📊 Performance Specifications

| Specification | Value |
|---------------|-------|
| **Max Speed** | Variable (60-120 PWM) |
| **Turn Radius** | Adjustable (0-110° servo range) |
| **Detection Range** | 250cm (ultrasonic) |
| **Camera FOV** | Enhanced with 0.45x wide-angle lens |
| **Processing Rate** | ~10fps image analysis |
| **Battery Life** | ~30-45 minutes continuous operation |
| **Weight** | Optimized with thin wheels and efficient design |

---

## 🏆 Team Information

**Team Name**: Team Paragon  
**Team Leader**: Abdullah Al Mahmud
**Competition**: World Robot Olympiad (WRO) 2023
**Category**: Future Engineers

---

## 🔧 Troubleshooting

### Common Issues
1. **Camera Not Detected**: Check CSI cable connections and camera permissions
2. **Serial Communication Failed**: Verify baud rate and port settings
3. **Motor Not Responding**: Check power connections and motor driver wiring
4. **Servo Jittery**: Calibrate center position and check power supply stability
5. **IMU Drift**: Recalibrate offsets and check mounting stability

### Debug Tools
- **Web Interface**: Real-time visualization of detection and decisions
- **Serial Monitor**: Arduino debugging and sensor readings
- **LED Indicators**: Status feedback for system states

---

## 📚 Technical Documentation

### Code Structure
```
Roam-E/
├── Arduino/                    # Arduino sketches
│   ├── final_round/           # IMU-based navigation
│   ├── final_round_wa/        # Wall avoidance variant
│   └── qualifier/             # Open round code
├── Source Code/
│   ├── Challenge Round/       # Python computer vision
│   └── Open Round/           # Arduino sensor-based
├── Mobility Management/       # Hardware documentation
├── Obstacle Management/       # AI algorithm documentation
├── Power & Sense Management/  # Circuit diagrams and power specs
└── Team & Vehicle Photos/     # Project images
```

### Key Algorithms
- **PID Control**: Wall following and distance maintenance
- **HSV Color Detection**: Traffic signal recognition
- **Blob Analysis**: Object detection and tracking
- **Serial Protocol**: Arduino-Jetson communication
- **State Machine**: Competition mode management

---

## 🚀 Future Improvements

- **Enhanced AI**: Deep learning for better object recognition
- **Sensor Fusion**: Combine ultrasonic and vision data
- **Path Planning**: Advanced route optimization algorithms
- **Energy Efficiency**: Power consumption optimization

---

For technical support or questions about the project, please refer to the documentation in each component folder or contact the development team.

---

*Happy Building! 🤖✨*

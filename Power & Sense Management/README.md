# Power and Sense Management

In this document, we delve into the critical aspects of power management and sensor selection in our vehicle project. We justify our choices of sensors, elucidating their roles and power consumption in the vehicle's operation. Additionally, we provide a professionally detailed wiring diagram and a Bill of Materials (BOM), covering all essential aspects of the electrical system's schematic representation. The source links for all sensors and power sources are listed below.

## Power Management

For efficient power management, we made the following considerations:

- **Power Source:** We utilized an 1800mAh LiPo battery, providing a current of 1.8 amperes. At an input voltage of 11.8 volts, this resulted in a power generation of 11.8 x 1.8 = 21.24 watts.

- **Voltage Boosting:** We employed the **LM2577 DC TO DC step-up buck converter** to increase the voltage to 13V, maximizing the output of the DC motor (25GA370). This conversion yielded 13V x 1.8A = 23.4 watts, surpassing the input power.

- **Voltage Regulation:** The **LM2596 Step Down Buck Converter** was used to power the Jetson Nano, servo motor, and Arduino Uno, generating an additional 5.1V x 1.8A = 9.18 watts of power.

- **Distribution:** Power was supplied from the Jetson Nano to the PyCam and then to the USB signal transferring component. The Arduino's 5V power supply operated the IMU sensor, generating 5V x 1.8A = 9 watts of power. The same 5V Arduino power supply was also used to power the Sonar sensors.

## Sensors

We selected sensors based on their suitability for navigating diverse challenges in our project:

- **HC-SR04 Ultrasonic Distance Sensor:** This sensor was programmed for use in the preliminary round, providing essential distance measurement capabilities.

- **Raspberry Pi Camera Module 3:** The Camera Module V3 serves as a versatile camera accessory for Raspberry Pi boards. Python libraries, such as picamera, enable easy control of camera settings, including resolution, frame rate, and exposure. This camera captures images and videos, making it a valuable tool for our project. It is enhanced with a wide-angle lens (0.45x) for a broader view, facilitating obstacle detection and image processing.

- **10-DOF IMU Sensor:** A 10-DOF IMU Sensor (Inertial Measurement Unit) provides precise motion and orientation measurements. It combines accelerometers, gyroscopes, and magnetometers for accurate tracking of the car's motion in three-dimensional space. We used the MPU9255 and BMP280 sensors connected to it to calculate angles and aid in vehicle control. The angle measurement ranges from 0 to 1080 degrees, with the vehicle programmed to stop at 1050 degrees.

## Processors

Our project involves two key processors:

- **Jetson Nano:** The Jetson Nano is responsible for capturing and processing visual data from the Pi Camera. It makes real-time navigation decisions based on image analysis and communicates these decisions to the Arduino for motor control. This continuous feedback loop allows the car to operate autonomously while reacting to its environment through image processing and AI algorithms.

- **Arduino Uno:** The Arduino Uno controls the physical components of the car, including motors, servos, and sensors. It receives commands from the Jetson Nano and adjusts motor speeds and directions accordingly. The Arduino also interfaces with various sensors, collecting data about obstacles and wheel rotation, and controls actuators like servos for steering the front wheels. It acts as the communication bridge between the Jetson Nano and the car's hardware components, ensuring real-time control for quick responses to changing conditions.

The Jetson Nano and Arduino Uno work in tandem to enable the self-running car to navigate autonomously and adapt to its environment effectively.

## Wiring Diagram

![image](https://github.com/zero-abd/baby_musks_wroBot/assets/48104263/29023387-f17b-4c62-b330-fb7910b5bb4a)

For any further inquiries, please feel free to reach out.

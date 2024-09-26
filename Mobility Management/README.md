# Mobility Management

In this document, we will provide an overview of the mobility management techniques employed in our vehicle project. We will explain the design of the vehicle's chassis, the integration of motors, and the critical aspects of its engineering. Additionally, we will offer assembly instructions and information on the availability of 3D CAD files for 3D printing specific components.

## Chassis and Motor Selection

To ensure optimal mobility, we carefully examined the design of the vehicle's chassis and selected the following components:

- **Chassis:** We chose the [4WD Smart Car Robot Chassis](https://www.elecrow.com/4wd-smart-car-robot-chassis-for-arduino-servo-steering.html) for its versatility and robustness.

- **Front Motor:** The front wheels are controlled by the **MG-996 360 Degree Servo Motor** known for its large torque and precision.

- **Back Motor:** Initially, we used the **JGA25-370** motor, which had lower torque. However, we later upgraded to the **25GA370 300RPM Metal Gear DC 12V** motor to enhance the vehicle's performance and prevent it from getting stuck.

## Motor Control Mechanisms

We employed the **BTS7960 Motor Drive Module** for motor control in our vehicle project. Here's how it works:

- **Back Wheels (25GA370 Motor):**
  - The BTS7960 Motor Drive Module receives control signals from the microcontroller.
  - For forward motion, it provides the appropriate voltage and current to the 25GA370 motor, causing it to rotate in the desired direction.
  - The vehicle's speed is determined by adjusting the supplied voltage and current.
  - For reverse motion, the polarity of the voltage supplied to the motor is reversed.

- **Front Wheels (MG-996 Servo Motor):**
  - The MG-996 Servo Motor is initially centered at 40 degrees, aligning the front wheels straight ahead.
  - To turn left, the microcontroller sends a command to the servo to set its angle to 0 degrees.
  - To turn right, the microcontroller sends a command to the servo to set its angle to 110 degrees, turning the front wheels accordingly.

By precisely controlling the motor and servo, we achieve smooth and accurate vehicle movement.

## Wheel Specifications

The wheels used in our project have a radius of 3.35 cm and a width of 54 mm, allowing for improved maneuverability. The advantages of these wheel specifications include:

- **Maneuverability:** Smaller wheels with thin widths enhance maneuverability in tight spaces.
- **Reduced Weight:** Thinner wheels are lighter, contributing to overall weight reduction and improved energy efficiency.
- **Efficiency:** Small radius wheels with thin widths are ideal for applications with limited space, enabling compact designs without sacrificing performance.
- **Enhanced Precision:** In certain applications, such as robotics or machining equipment, thin wheels provide greater precision and control due to reduced ground contact.

## Chassis Mounting and Accessories

The selected chassis came with a range of accessories, which we used for mounting and enhancing our vehicle:

- *List of Accessories:* (For details, refer to [this link](https://www.elecrow.com/4wd-smart-car-robot-chassis-for-arduino-servo-steering.html))
  - 65 MM wheels x4
  - Metal chassis x 1
  - Acrylic chassis x 2
  - Anti-collision cotton x1
  - Metal gear motor x1
  - Engine counting bracket x1
  - Steering wheel x 2
  - Rear-wheel link x1
  - Rolling bearing flange x2
  - 17 MILLIMETERS D-5MM coupling x2
  - MG996R actuator x1
  - Servo arm x1
  - L Mount bracket x1
  - Sector x 2
  - Pull rod x2
  - Steering bellcrank x2
  - Transmission link x2
  - Bearing (8mm) x 2
  - Bearing (12mm) x 2
  - M2*10mm Aluminum column x 3
  - Hexagon adapter x 2
  - Switch x1
  - M3 * 16mm passage double copper column x 4
  - M3 * 22mm passage double copper column x 8
  - M3 * 35mm passage double copper column x 4
  - M3 * 5mm head screwdriver x3
  - M3 * 5mm screwdriver x15
  - M3 * 8mm screwdriver x40
  - M2.5 * 10mm Screw x 10
  - M2.5 locknut x 2
  - M4 locknut x 3
  - Gasket x4
  - Cross sleeve x1
  - Cable ties x6
  - 22AWG wire x 3
  - Heat shrink tube x1
  - Tool x1

## Assembly and Configuration

After assembling the entire metallic chassis, we mounted the motors at the front and rear. We also integrated three HC-05 sonar sensors – one on the right, one on the left, and one at the center. The back section houses the buck converters and Arduino, while the middle compartment contains the LiPo battery and Jetson Nano.

## 3D Printing and Building Instructions

For building instructions and 3D printing designs, please refer to the [3D designs submission](insert_link_here) section. It contains detailed instructions for creating and assembling components using 3D printed parts and other materials.

For any further assistance or inquiries, please feel free to reach out.

Happy building!

![Line Following Transport Robot](car.jpg)

# Line Following Transport Robot

## Description

This program runs a line-following transport robot that uses infrared (IR) sensors for navigation and a servo motor for steering control. It's designed to follow a predetermined path marked by a line, performing a pick up and unloading operation of an objects at designated points. The robot utilizes a PID controller for precise steering adjustments based on feedback from the IR sensors.


## Hardware Requirements

- Arduino Mega.
- Adafruit Motor Shield for controlling DC motors.
- Servo motor for steering control.
- Fischertechnik IR sensors for line detection and navigation.
- DC motors for driving and transport mechanisms.

## Software Requirements

- Arduino IDE for compiling and uploading the firmware to the Arduino board.

## Installation Instructions

### 1. Install Arduino IDE

Download and install the Arduino IDE from [the official Arduino website](https://www.arduino.cc/en/software).

### 2. Install Required Libraries

The project requires the following libraries:
- `AFMotor.h` for controlling the Adafruit Motor Shield.
- `Servo.h` for controlling the servo motor.
- `PID_v1.h` for implementing the PID controller.

To install these libraries, follow these steps:

**AFMotor Library:**
1. Visit the [Adafruit Motor Shield library repository](https://github.com/adafruit/Adafruit-Motor-Shield-library) on GitHub.
2. Download the repository as a ZIP file.
3. Open the Arduino IDE, navigate to `Sketch` > `Include Library` > `Add .ZIP Library...` and select the downloaded ZIP file.

**Servo Library:**
- The `Servo` library comes pre-installed with the Arduino IDE.

**PID Library:**
1. Open the Arduino IDE.
2. Go to `Sketch` > `Include Library` > `Manage Libraries...`.
3. In the Library Manager, search for "PID" and find the "PID by Brett Beauregard".
4. Click `Install`.

### 3. Load and Upload the Sketch
First, clone repository
```
git clone https://git.tu-berlin.de/line-following-robot/line_following_robot.git
```
Next, open Arduino IDE and compile and upload code
## Operation Instructions

After uploading the sketch, the robot will wait for 2 seconds before entering  the `DRIVE_STEER` state, following the line. It will automatically transition through `PICKUP`, `RETRACT`, `UNLOAD`, and `STOP` states based on the line count, performing actions as programmed.





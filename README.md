


## Connection

# 3-Wheeled Robot Control System
![image](https://github.com/ARRRsunny/3-omni-wheels-car/blob/main/3_wheel_robot.jpg)
This Arduino project controls a 3-wheeled robot capable of omni-directional movement. It uses a Bluetooth or serial communication module to receive commands, which control the robot's motors and trigger additional actions. The robot can move forward, backward, diagonally, rotate, and perform special movements using different motor speed combinations.

## Features

- **Omni-directional Movement**: Supports forward, backward, diagonal, and rotational movements.
- **Bluetooth/Serial Control**: Commands are received via software serial communication (pins 9 and 8).
- **PWM Motor Speed Control**: Each motor's speed and direction are independently controlled using PWM and direction pins.
- **Trigger Activation**: Additional functionality (e.g., activating a shooter or other mechanism) via a trigger pin.

## Hardware Requirements

- **Microcontroller**: Arduino Uno/NANO (or compatible board).
- **Motors**: 3 DC motors with motor drivers.
- **Motor Driver Module**: Supports PWM control and direction for at least 3 motors. using L298N 4-Channel motor driver.
- **Bluetooth Module**: E.g., HC-05 or HC-06 (or any serial communication module).
- **Power Supply**: Suitable for the motors and Arduino.
- **Additional Components**:
  - Wires and connectors.
  - 1 trigger mechanism (optional).

## Pin Configuration

### Motor Pins

| Motor | EN (PWM) | IN1  | IN2  |
|-------|----------|------|------|
| Motor 1 | Pin 5   | Pin 6 | Pin 7 |
| Motor 2 | Pin 3   | Pin 2 | Pin 4 |
| Motor 3 | Pin 11  | Pin 10 | Pin 12 |

![image](https://github.com/ARRRsunny/3-omni-wheels-car/blob/main/motordriver.jpg)
![image](https://github.com/ARRRsunny/3-omni-wheels-car/blob/main/pin.png)
### Trigger Pin
- **Trigger_PIN**: Pin 13

### Bluetooth/Serial
- **RX**: Pin 9
- **TX**: Pin 8

## Software Requirements

- **Arduino IDE**: Version 1.8.0 or higher.
- **Libraries**:
  - `Servo.h` (pre-installed in Arduino IDE).
  - `SoftwareSerial.h` (pre-installed in Arduino IDE).

## Setup Instructions

1. **Hardware Setup**:
   - Connect the motors to the motor driver module according to the pin configuration table.
   - Connect the motor driver module to the Arduino.
   - Connect the Bluetooth module to pins 9 (RX) and 8 (TX).
   - Connect the trigger mechanism to pin 13 (if used).
   - Provide appropriate power to the motors and the Arduino.

2. **Upload the Code**:
   - Open the Arduino IDE and copy the provided sketch.
   - Select the correct board and port in the Arduino IDE.
   - Upload the code to your Arduino board.

3. **Testing**:
   - Pair the Bluetooth module with your smartphone or computer.
   - Send commands via a Bluetooth terminal app or serial monitor.
   - Observe the robot's movements according to the commands.

## Command List

| Command | Action                          |
|---------|---------------------------------|
| `0`     | Stop all motors                |
| `1`     | Move forward                   |
| `2`     | Move backward                  |
| `3`     | Special horizontal movement 1  |
| `4`     | Move diagonally left           |
| `5`     | Move diagonally right          |
| `6`     | Special horizontal movement 2  |
| `7`     | Rotate clockwise               |
| `8`     | Rotate counterclockwise        |
| `R`     | Rotate clockwise at speed 190  |
| `L`     | Rotate counterclockwise at speed 190 |
| `K`     | Activate the trigger           |
| `B`     | Deactivate the trigger         |

## Code Overview

- **`drive_motor()`**: Controls the speed and direction of all three motors.
- **`case_sw()`**: Maps received commands to specific robot movements or actions.
- **`hori_move()`**: Handles special horizontal movements by alternating motor speeds.
- **`setMotorPinState()`**: Sets the direction of a motor based on the speed value.
- **`setup()`**: Initializes the motors, pins, and serial communication.
- **`loop()`**: Continuously checks for incoming commands and executes the corresponding action.

## Example Usage

1. **Send Command `1`**:
   - Robot moves forward.
   - Motor 2 moves backward, Motor 3 moves forward, Motor 1 stops.
   
2. **Send Command `R`**:
   - Robot rotates clockwise.
   - All motors move forward at a speed of 190.

3. **Send Command `K`**:
   - Trigger pin is activated (set to HIGH).

## Troubleshooting

- **Robot not moving**:
  - Check motor connections and power supply.
  - Ensure the Bluetooth module is properly paired with your device.
  - Verify the command being sent matches the expected format (`'0'` to `'8'`, `'R'`, `'L'`, `'K'`, `'B'`).
  
- **Unexpected behavior**:
  - Ensure the motor driver and wiring match the pin configuration.
  - Verify the logic in `drive_motor()` and `setMotorPinState()`.

## Future Improvements

- Add support for dynamic speed control via additional commands.
- Implement non-blocking code for smoother movements using `millis()`.
- Add feedback from the robot (e.g., motor status or sensor data) to the controller.

## License

This project is open-source and free to use. Modify and distribute as needed, but ensure proper attribution to the original author.

---

**Happy Building!**

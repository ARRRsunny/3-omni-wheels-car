
# 3-Wheeled Omni Robot Control System

![image](https://github.com/ARRRsunny/3-omni-wheels-car/blob/main/3_wheel_robot.jpg)

This repository contains Arduino sketches for controlling a 3-wheeled omni-directional robot. The latest sketch is `new_pcb/PWM_brench_ble.ino`, which uses PWM motor control and Bluetooth/serial command input to drive the robot and activate an optional trigger output.

## Latest Sketch

- `new_pcb/PWM_brench_ble.ino` – newest board version with SoftwareSerial-based Bluetooth input and PWM motor outputs.
- `new_pcb/brench_ble.ino` – alternate sketch using the Arduino hardware serial port at 115200 baud.
- `new_pcb/brench.ino` – Bluetooth sketch using `SoftwareSerial`.

## Features

- Omni-directional movement: forward, backward, diagonal, and rotation.
- Bluetooth/serial command control.
- Independent PWM control for 3 motors.
- Trigger output on pin 13.
- Special horizontal motion routines for commands `3` and `7`.
- Built-in debug output over serial.

## Hardware Requirements

- Arduino Uno / Nano or compatible board.
- 3 DC motors with a compatible motor driver (e.g. L298N or similar).
- Bluetooth module such as HC-05 / HC-06 for wireless control.
- Motor power supply and Arduino power source.
- Wires, connectors, and optional trigger mechanism.

## Pin Configuration

### Motor Pins

| Motor   | EN (PWM) | IN1 | IN2 |
|--------|----------|-----|-----|
| Motor 1 | 11       | 12  | 10  |
| Motor 2 | 5        | 7   | 6   |
| Motor 3 | 3        | 4   | 2   |

![image](https://github.com/ARRRsunny/3-omni-wheels-car/blob/main/motordriver.jpg)
![image](https://github.com/ARRRsunny/3-omni-wheels-car/blob/main/pin.png)

### Trigger Pin

- `Trigger_PIN`: 13

### Bluetooth / Serial

- `RX`: 9
- `TX`: 8

> For `new_pcb/PWM_brench_ble.ino` and `new_pcb/brench.ino`, Bluetooth uses `SoftwareSerial` on pins 9/8. For `new_pcb/brench_ble.ino`, commands are read from the hardware `Serial` port at `115200`.

## Software Requirements

- Arduino IDE 1.8.0 or higher.
- Built-in libraries:
  - `Servo.h`
  - `SoftwareSerial.h`

## Upload Instructions

1. Open the Arduino IDE.
2. Load `new_pcb/PWM_brench_ble.ino` for the latest Bluetooth/PWM sketch.
3. Select the correct board and USB port.
4. Upload the sketch.

## Operation

1. Power the Arduino and the motor driver.
2. Pair the Bluetooth module with your phone or computer (if using Bluetooth).
3. Send command characters from a Bluetooth terminal app or serial monitor.
4. The sketch prints the current command to serial for debugging.

## Command List

| Command | Action |
|---------|--------|
| `0` | Stop all motors |
| `1` | Move forward |
| `2` | Move backward |
| `3` | Special horizontal movement 1 |
| `4` | Move diagonally left |
| `5` | Move diagonally right |
| `6` | Move diagonally right / special move |
| `7` | Special horizontal movement 2 |
| `8` | Move diagonally left / rotate motion |
| `R` | Rotate clockwise at fixed speed |
| `L` | Rotate counterclockwise at fixed speed |
| `K` | Activate trigger output |
| `B` | Deactivate trigger output |

## Code Behavior

- `drive_motor(v1, v2, v3)` sets motor directions and PWM values.
- `case_sw(movement)` selects movement based on received command.
- `hori_move(3)` and `hori_move(7)` execute two-step horizontal motion sequences.
- `setMotorPinState()` controls motor driver direction pins from the sign of each speed value.
- `loop()` polls serial input every 20 ms and applies the current movement command.
- Invalid input defaults to `0` (stop).

## Example Commands

- `1`: forward motion with motor 1 stopped, motor 2 reversed, motor 3 forward.
- `R`: clockwise rotation at a fixed speed value of `150` for all motors.
- `K`: set trigger pin 13 HIGH.

## Troubleshooting

- Robot does not move:
  - Confirm power to the motor driver and motors.
  - Verify wiring matches the pin table.
  - Check Bluetooth pairing or serial connection.
  - Make sure commands are single characters from the supported set.

- Unexpected motion:
  - Check the motor driver direction wiring.
  - Ensure `SoftwareSerial` pins 9/8 are not conflicting with other hardware.

## Notes

- The code prints the active movement character on every loop for debugging.
- Special movement commands `3` and `7` use short `delay(15)` steps to alternate motor outputs.
- The sketch uses absolute PWM values up to `255` for full speed and a reduced constant `150` for rotation commands.

## License

This project is open source. Feel free to use, modify, and share the sketches with attribution.

# 3-Wheeled Omni Robot Control System

![image](https://github.com/ARRRsunny/3-omni-wheels-car/blob/main/asset/3_wheel_robot.jpg)

This repository contains Arduino sketches for controlling a 3-wheeled omni-directional robot. The system utilizes PWM motor control, Bluetooth/serial command input, and PID control for precise movement. It supports forward, backward, diagonal, rotational, and special horizontal movements, along with an optional trigger output.

## Project Sketches

* `new_pcb/PWM_brench_ble.ino`: For Arduino Nano BLE, focuses on PWM motor outputs (Latest).
* `new_pcb/MPU6050_brench_ble.ino`: For Arduino Nano BLE, utilizes PID control.
* `new_pcb/brench_ble.ino`: For Arduino Nano BLE, an alternate sketch using the hardware serial port at 115200 baud.
* `new_pcb/brench.ino`: Standard Arduino Nano Bluetooth sketch using `SoftwareSerial`.

## Features

* Omni-directional movement: forward, backward, diagonal, and rotation.
* Wireless control via Bluetooth or serial command.
* Independent PWM control for 3 motors.
* PID control for stabilized movement.
* Special horizontal motion routines (commands `3` and `7`).
* Digital trigger output on pin 13.
* Built-in debug output over serial.

## Hardware Requirements

* Arduino Uno / Nano or compatible board.
* Custom PCB (new or old version).
* 3 DC motors with a compatible motor driver (e.g., L298N).
* Gyroscope module (e.g., MPU6050 or MPU9250).
* Bluetooth module(e.g., HC-05 / HC-06) / BLE (Bluetooth Low Energy) for wireless control. Or, 
* Dedicated motor power supply and Arduino power source.
* Wires, connectors, and an optional trigger mechanism.

## Software Requirements

* Arduino IDE 1.8.0 or higher.
* `Servo.h` (Built-in, for PWM output).
* `SoftwareSerial.h` (Built-in, for UART communication).
* `Wire.h` (Built-in, for I2C communication).
* `MPU6050_light.h` (Third-party, for MPU6050 gyroscope).

---

## Pin Configuration

### Motor Driver Pins

| Motor | EN (PWM) | IN1 | IN2 |
| --- | --- | --- | --- |
| Motor 1 | 11 | 12 | 10 |
| Motor 2 | 5 | 7 | 6 |
| Motor 3 | 3 | 4 | 2 |

### Sensors & Communication Pins

| Component | Pin | Notes |
| --- | --- | --- |
| Trigger Output | 13 | Digital HIGH/LOW |
| Bluetooth RX | 9 | `SoftwareSerial` only (`new_pcb/brench.ino`) |
| Bluetooth TX | 8 | `SoftwareSerial` only (`new_pcb/brench.ino`) |
| I2C SDA | A4 | For MPU6050 Gyroscope |
| I2C SCL | A5 | For MPU6050 Gyroscope |

> **Note:** For `new_pcb/brench_ble.ino`, commands are read from the hardware `Serial` port at `115200` baud. For the gyroscope, please keep the robot perfectly still for a few seconds after powering on to allow for calibration. 

---

## Setup & Operation

### Upload Instructions

1. Open the Arduino IDE.
2. Load your desired sketch (e.g., `new_pcb/brench_ble.ino`).
3. Select the correct board and USB port.
4. Upload the sketch.

### Operating the Robot

1. Power the Arduino and the motor driver (ensure battery packs are switched on).
2. Pair the Bluetooth module with your phone or computer (if using wireless).
3. Send command characters from a Bluetooth terminal app or serial monitor.
4. Observe the serial monitor; the sketch prints the current command for debugging.

---

## Command List

| Command | Action |
| --- | --- |
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
| `K` | Activate trigger output (Pin 13 HIGH) |
| `B` | Deactivate trigger output (Pin 13 LOW) |

### Example Commands

* **`1`**: Forward motion (Motor 1 stopped, Motor 2 reversed, Motor 3 forward).
* **`R`**: Clockwise rotation at a fixed speed value of `150` for all motors.
* **`K`**: Sets trigger pin 13 HIGH.

---

## Code Behavior

* `drive_motor(v1, v2, v3)`: Sets motor directions and PWM values.
* `case_sw(movement)`: Selects movement behavior based on the received command.
* `hori_move(3)` / `hori_move(7)`: Executes two-step horizontal motion sequences using short `delay(15)` intervals to alternate motor outputs.
* `setMotorPinState()`: Controls motor driver direction pins based on the sign (+/-) of each speed value.
* `loop()`: Polls serial input every 20 ms and applies the current movement command. Invalid inputs default to `0` (stop).
* **Speed Constraints:** The sketch uses absolute PWM values up to `255` for full speed, and a reduced constant of `150` for rotation commands.

---

## Troubleshooting

| Issue | Possible Solutions |
| --- | --- |
| **No movement at all** | Check the Bluetooth or Serial connection. The HC-05 slowly blinking means it is connected; Nano-BLE keeping flashing means connected. |
| **No movement, but powered and connected** | Ensure the battery pack is turned on. Re-upload the Arduino program. Check or replace the PCB, Arduino, or Bluetooth module. |
| **One motor does not move** | Check the wiring to that motor. Inspect or replace the PCB. Confirm power is reaching the motor driver and motors. |
| **Unexpected motion** | Check the motor driver direction wiring. Ensure `SoftwareSerial` pins 9/8 are not conflicting with other hardware. Verify your wiring matches the pin table. |
| **Cannot upload program** | Try selecting "Arduino UNO" in the IDE even if using a Nano. Try another USB port or cable. Upload a blank program to reset the board (see below). |

> **Blank Program for Resetting:**
> ```cpp
> void setup() {
>   // run once:
> }
> void loop() {
>   // run repeatedly:
> }
> 
> ```
> 
> 

## License

This project is open source. Feel free to use, modify, and share the sketches with attribution.

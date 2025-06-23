# FCS
---
## Project Objective

Fire Control System (FCS) & Visual Verification of Intermittent Target Tracking  
Objective: To use tracking and measurement values in dynamic situations as reference points for targeting.

---

## Components

### Sensors
- Pixy2 (Camera Sensor)
- VL53L1X (Distance Measurement Sensor)

### Motors
- SG90 (2 units)
- MG996R (1 unit)
- Generic DC Motor (4 units)

### Others
- KY-008 (Laser Module)
- HC-05 (Bluetooth Module)
- Piezo Buzzer
- L298N (Motor Driver)

---

## Code Structure

### 1. Master Board
1) Waits to receive a signal via Bluetooth.
2) Upon receiving a signal via Bluetooth, executes the corresponding command:  
  - DC Motor On & Off  
  - Laser Module On & Off  
  - Status = 0 (Stop) / 1 (Forward) / 2 (Moving and Shooting) / 3 (Stop and Targeting)
3) Sends the received command signal to the Slave board using SPI communication.
4) When the Slave board sends a targeting completion signal, receives an interrupt and inputs a PWM signal to the piezo buzzer.

### 2. Slave Board
1) Receives command signals from the Master board via SPI and executes the corresponding command - automatic targeting On & Off.
2) If the target is detected, obtains the target's position from the Pixy2 sensor.
3) Controls the servo motors in the direction of the target and "Sensor Module Servo Motor" to align the Pixy2 sensor's center with the target.
4) Using the VL53L1X sensor, calculates the rotation angle of the "Sensor Module Servo Motor" based on the measured distance.
5) Controls the laser module (KY-008) to target the objective.
6) When targeting is complete and the lock-on flag is activated, sends the targeting completion signal to the Master board.

---

## Results

![Tank 1](FCS1.mp4)

![Tank 2](FCS2.mp4)

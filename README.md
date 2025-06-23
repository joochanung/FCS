# FCS
프로젝트 목표 : 사격통제장치(FCS) 및 기동간사격
시연 목표 : 움직이는 상황에서의 트래킹 및 측정값을 바탕으로 목표물 조준

구성 :
센서 
1. Pixy2 ( 카메라 센서 )
2. VL53L1X ( 거리 측정 센서 )

모터
1. SG90 2EA
2. MG996R 1EA
3. 기어드 DC모터 4EA

기타
1. KY-008 ( 레이저 모듈 )
2. HC-05 ( 블루투스 모듈 )
3. Piezo buzzer ( 피에조 부저 )
4. L298N ( 모터 드라이버 )

코드 구성
1. Master Board
  1) 블루투스를 통해 신호를 받을때까지 대기
  2) 블루투스에서 신호를 받으면 그에 맞는 명령 수행
    - DC모터 On & Off / 레이저 모듈 On & Off
    - status = 0(정지) / 1(전진) / 2(기동간사격) / 3(정지 및 조준)
  3) 받은 블루투스 신호를 SPI 통신을 이용해 Slave 보드로 전달
  4) Slave 보드가 조준 완료 신호를 보내면 Interrupt를 받아서 PWM 신호를 피에조 부저에 입력

2. Slave Board
  1) SPI 통신을 통해 블루투스 신호를 전달 받으면 그에 맞는 명령 수행
    - 자동 조준 On & Off
  2) 목표물이 탐지되면 Pixy2 센서에서 목표물의 위치값 전달
  3) 전달받은 위치값을 바탕으로 "포탑 회전 서보모터"와 "센서 모듈 서보모터"를 제어하여 Pixy2 센서의 중앙에 목표물이 위치하도록 함
  4) VL53L1X가 측정한 목표물과의 거리와 "센서 모듈 서보모터"의 회전각을 계산
  5) 이를 통해 "포신 상하 서보모터 (레이저 모듈)"를 제어하여 목표물을 조준하도록 함
  6) 목표물을 조준 완료하면 lockon flag가 활성화되고, 조준 완료 신호를 Master 보드로 전달

---
## Project Objective

Fire Control System (FCS) & Visual Verification of Intermittent Target Tracking  
Objective: To use tracking and measurement values in dynamic situations as reference points for targeting.

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

## Code Structure

### 1. Master Board
1) Waits to receive a signal via Bluetooth.
2) Upon receiving a signal via Bluetooth, executes the corresponding command:  
  - DC Motor On & Off  
  - Laser Module On & Off  
  - Status = 0 (Stop) / (Forward) / (Backward) / (Left) / (Right) / (Targeting)
3) Sends the received command signal to the Slave board using SPI communication.
4) When the Slave board sends a targeting completion signal, receives an interrupt and inputs a PWM signal to the piezo buzzer.

### 2. Slave Board
1) Receives command signals from the Master board via SPI and executes the corresponding command - automatic targeting On & Off.
2) If the target is detected, obtains the target's position from the Pixy2 sensor.
3) Controls the servo motors in the direction of the target and "Sensor Module Servo Motor" to align the Pixy2 sensor's center with the target.
4) Using the VL53L1X sensor, calculates the rotation angle of the "Sensor Module Servo Motor" based on the measured distance.
5) Controls the laser module (KY-008) to target the objective.
6) When targeting is complete and the lock-on flag is activated, sends the targeting completion signal to the Master board.

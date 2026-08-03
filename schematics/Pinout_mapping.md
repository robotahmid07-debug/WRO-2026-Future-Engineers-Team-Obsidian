# Pin Connections 
## Raspberry Pi 5 

| Physical Pin | Pi Function / GPIO | Connected Device | Connected Device Pin | Protocol |
| :--- | :--- | :--- | :--- | :--- |
| **Pin 1** | 3.3V Power | **HuskyLens** | VCC | PD |
| **Pin 3** | GPIO 2 (SDA) | **HuskyLens** | SDA | I2C |
| **Pin 5** | GPIO 3 (SCL) | **HuskyLens** | SCL | I2C |
| **Pin 6** | GND | **HuskyLens** | GND | Common Ground |
| **Pin 8** | GPIO 14 (TX) | **ESP32-S3** | GPIO 17 (RX) | UART |
| **Pin 10** | GPIO 15 (RX) | **ESP32-S3** | GPIO 18 (TX) | UART |
| **Pin 15** | GPIO 22 | **Mode Switch_1** | Pin A | Digital Input |
| **Pin 31** | GPIO 26 | **Mode Switch_2** | Pin B | Digital Input |
| **USB Port 1** | USB Data/Power | **RPLiDAR C1** | USB Connector | LiDAR Data & Power |

---
## ESP32-S3 

| ESP32-S3 Pin | Connected Device | Connected Device Pin | Protocol |
| :--- | :--- | :--- | :--- |
| **GPIO 17** | Raspberry Pi 5 | Pin 8 (GPIO 14 TX) | UART |
| **GPIO 18** | Raspberry Pi 5 | Pin 10 (GPIO 15 RX) | UART |
| **GPIO 4** | HC-SR04 (Front) | Trig | Digital Output |
| **GPIO 5** | HC-SR04 (Front) | Echo | Digital Input |
| **GPIO 6** | HC-SR04 (Front-Left) | Trig | Digital Output |
| **GPIO 7** | HC-SR04 (Front-Left) | Echo | Digital Input |
| **GPIO 15** | HC-SR04 (Front-Right) | Trig | Digital Output |
| **GPIO 16** | HC-SR04 (Front-Right) | Echo | Digital Input |
| **GPIO 1** | BTS7960 Motor Driver | RPWM | PWM Signal (Forward) |
| **GPIO 2** | BTS7960 Motor Driver | LPWM | PWM Signal (Reverse) |
| **TBD / 3.3V** | BTS7960 Motor Driver | R_EN + L_EN | Digital Output |
| **GPIO 41** | Steering Servo | Signal Pin | PWM Steering Signal |
| **GPIO 8** | BNO086 IMU | SDA | I2C |
| **GPIO 9** | BNO086 IMU | SCL | I2C |

---
## BNO086 IMU 

| BNO086 Pin | Connected To | Protocol |
| :--- | :--- | :--- |
| **VIN** | ESP32-S3 3.3V | Power |
| **GND** | ESP32-S3 GND | Common Ground |
| **SDA** | ESP32-S3 GPIO 8 | I2C |
| **SCL** | ESP32-S3 GPIO 9 | I2C |

---
## BTS7960 Motor Driver

| Terminal / Pin | Connected To | Voltage / Signal Type | Description |
| :--- | :--- | :--- | :--- |
| **RPWM** | ESP32-S3 GPIO 1 | PWM Input | Forward Speed Control |
| **LPWM** | ESP32-S3 GPIO 2 | PWM Input | Reverse Speed Control |
| **R_EN** | ESP32 GPIO (TBD) / 3.3V | Digital High | Forward Enable |
| **L_EN** | ESP32 GPIO (TBD) / 3.3V | Digital High | Reverse Enable |
| **VCC** | ESP32 / External 5V | 5V DC | Driver Logic Power |
| **GND** | System Common Ground | Ground | Common Ground |
| **B+ (VMotor)** | LiPo Battery (+) | 11.1V Nominal | High Current Power Input |
| **B-** | LiPo Battery (-) | Battery Power GND | High Current Ground |
| **M+ / M-** | DC Drive Motor | High Current PWM | Motor Output Terminals |

---
## Steering Servo & Switches
### Steering Servo

| Servo Wire | Connected To | Description |
| :--- | :--- | :--- |
| **Signal** | ESP32-S3 GPIO 41 | PWM Signal |
| **VCC** | External BEC | Dedicated 6V Power (Buck Conv.) |
| **GND** | Common Ground | Power Ground |

### Switch 
#### Switch_1 
| Switch Pin | Connection Point | Logic / State |
| :--- | :--- | :--- |
| **Pin A** | Raspberry Pi 5 Pin 15 (GPIO 22) | **CLOSED (GND)** = Obstacle Challenge |
| **

| Switch Pin | Connection Point | Logic / State |
| :--- | :--- | :--- |
| **Pin A** | Raspberry Pi 5 Pin 15 (GPIO 22) | **CLOSED (GND)** = Obstacle Challenge |
| **Pin B** | Any Raspberry Pi GND Pin | **OPEN (High)** = Open Challenge |

---
### Hardware Architecture Flowchart
```mermaid
graph TD
    subgraph Vision & Processing
        Pi[Raspberry Pi 5]
        Husky[HuskyLens Camera]
        Lidar[RPLiDAR C1]
    end
    subgraph Real-Time Control
        ESP[ESP32-S3 MCU]
        IMU[BNO086 IMU]
        Sonar[HC-SR04 x3]
    end
    subgraph Actuation & Power
        Driver[BTS7960 Driver]
        Motor[Drive Motor]
        Servo[Steering Servo]
        Battery[11.1V LiPo Battery]
    end
    Lidar -- USB Data --> Pi
    Husky -- I2C --> Pi
    Pi -- UART TX/RX --> ESP
    ESP -- I2C --> IMU
    ESP -- Digital I/O --> Sonar
    
    ESP -- PWM Signal --> Driver
    ESP -- PWM Signal --> Servo
    
    Battery -- High Current --> Driver
    Driver -- Power Output --> Motor

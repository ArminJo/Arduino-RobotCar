# Autonomous driving robot car

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://github.com/ArminJo/Arduino-RobotCar/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Arduino-RobotCar/actions)
![Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_Arduino-RobotCar)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FArduino-RobotCar)](https://github.com/brentvollebregt/hit-counter)

Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the environment.
Manual control is implemented by a GUI using a Bluetooth HC-05 Module and the BlueDisplay library.

Just overwrite the function doUserCollisionDetection() to test your own skill.

You may also overwrite the function fillAndShowForwardDistancesInfo(), if you use your own scanning method.

# Compile options / macros for this software
To customize the software to different car extensions, there are some compile options / macros available.<br/>
Modify it by commenting them out or in, or change the values if applicable. Or define the macro with the -D compiler option for global compile (the latter is not possible with the Arduino IDE, so consider using [Sloeber](https://eclipse.baeyens.it).<br/>
Compile options for the used **PWMMotorControl library** like `USE_ENCODER_MOTOR_CONTROL` are described [here](https://github.com/ArminJo/PWMMotorControl#compile-options--macros-for-this-library).

| Option | Default | File | Description |
|-|-|-|-|
| `USE_LAYOUT_FOR_NANO` | disabled | RobotCar.h | Use different pinout for Nano board. It has A6 and A7 available as pins. |
| `CAR_HAS_4_WHEELS` | disabled | RobotCar.h | Use modified formula dor turning the car. |
| `USE_US_SENSOR_1_PIN_MODE` | disabled | RobotCar.h | Use modified HC-SR04 modules or HY-SRF05 ones.</br>Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger pin. |
| `CAR_HAS_IR_DISTANCE_SENSOR` | disabled | RobotCar.h | Use Sharp GP2Y0A21YK / 1080 IR distance sensor. |
| `CAR_HAS_TOF_DISTANCE_SENSOR` | disabled | RobotCar.h | Use VL53L1X TimeOfFlight distance sensor. |
| `DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN` | disabled | Distance.h | The distance servo is mounted head down to detect even small obstacles. The Servo direction is reverse then. |
| `CAR_HAS_CAMERA` | disabled | RobotCar.h | Enables the `Camera` button for the `PIN_CAMERA_SUPPLY_CONTROL` pin. |
| `CAR_HAS_LASER` | disabled | RobotCar.h | Enables the `Laser` button for the `PIN_LASER_OUT` / `LED_BUILTIN` pin. |
| `CAR_HAS_PAN_SERVO` | disabled | RobotCar.h | Enables the pan slider for the `PanServo` at the `PIN_PAN_SERVO` pin. |
| `CAR_HAS_TILT_SERVO` | disabled | RobotCar.h | Enables the tilt slider for the `TiltServo` at the `PIN_TILT_SERVO` pin. |
| `ENABLE_RTTTL` | undefined | RobotCar.h | Plays melody after initial timeout has reached. Enables the Melody button, which plays a random melody. |
| `MONITOR_VIN_VOLTAGE` | disabled | RobotCar.h | Shows VIN voltage and monitors it for undervoltage. VIN/11 at A2, 1MOhm to VIN, 100kOhm to ground. |
| `VIN_VOLTAGE_CORRECTION` | undefined or 0.8 for UNO | RobotCar.h | Voltage to be subtracted from VIN voltage for voltage monitoring. E.g. if there is a series diode between LIPO and VIN as on the UNO boards, set it to 0.8. |
| `SUPPORT_EEPROM_STORAGE` | disabled | RobotCar.h | Activates the buttons to store compensation and drive speed. |

### Modifying compile options with Arduino IDE
First, use *Sketch > Show Sketch Folder (Ctrl+K)*.<br/>
If you did not yet stored the example as your own sketch, then you are instantly in the right library folder.<br/>
Otherwise you have to navigate to the parallel `libraries` folder and select the library you want to access.<br/>
In both cases the library files itself are located in the `src` directory.<br/>

### Modifying compile options with Sloeber IDE
If you are using Sloeber as your IDE, you can easily define global symbols with *Properties > Arduino > CompileOptions*.<br/>
![Sloeber settings](https://github.com/ArminJo/ServoEasing/blob/master/pictures/SloeberDefineSymbols.png)

# Wall detection
The problem of the ultrasonic values is, that you can only detect a wall with the ultrasonic sensor if the angle of the wall relative to sensor axis is approximately between 70 and 110 degree.
For other angels the reflected ultrasonic beam can not not reach the receiver which leads to unrealistic great distances.<br/>
Therefore I take samples every 18 degrees and if I get 2 adjacent short (<DISTANCE_FOR_WALL_DETECT) distances, I assume a wall determined by these 2 samples.
The (invalid) values 18 degrees right and left of these samples are then extrapolated by computeNeigbourValue().

# Pictures
2 wheel car with encoders, 2 LiPo batteries, Adafruit Motor Shield V2, Bluetooth connection, and servo mounted head down
![2 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/2WheelDriveCar.jpg)
4 wheel car, like 2 WD car before, but with servo mounted head up.
![4 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/4WheelDriveCar.jpg)
Encoder fork sensor
![Encoder fork sensor](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ForkSensor.jpg)
Servo mounted head down
![Servo mounting](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ServoAtTopBack.jpg)
VIN sensing
![VIN sensing](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/SensingVIn.jpg)

# SCREENSHOTS
Start page
![MStart page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/HomePage.png)
Test page
![Manual control page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/TestPage.png)
Automatic control page with detected wall at right
![Automatic control page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/AutoDrivePage.png)
- Green bars are distances above 1 meter or above double distance of one ride per scan whichever is less.
- Red bars are distanced below the distance of one ride per scan -> collision during next "scan and ride" cycle if obstacle is ahead.
- Orange bars are the values between the 2 thresholds.
- The tiny white bars are the distances computed by the doWallDetection() function. They overlay the green (assumed timeout) values.
- The tiny black bar is the rotation chosen by doCollisionDetection() function.


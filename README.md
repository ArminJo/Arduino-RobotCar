# Autonomous driving robot car

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://github.com/ArminJo/Arduino-RobotCar/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Arduino-RobotCar/actions)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FArduino-RobotCar)](https://github.com/brentvollebregt/hit-counter)

Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the environment.
Manual control is implemented by a GUI using a Bluetooth HC-05 Module and the BlueDisplay library.

Just overwrite the function doUserCollisionDetection() to test your own skill.

You may also overwrite the function fillAndShowForwardDistancesInfo(), if you use your own scanning method.

# Compile options

| Option | Default | File | Description |
|-|-|-|-|
| `USE_ENCODER_MOTOR_CONTROL` | disabled | PWMDCMotor.h | Use fork light barrier and an attached encoder disc to enable motor distance and speed sensing. |
| `USE_ADAFRUIT_MOTOR_SHIELD` | disabled | PWMDcMotor.h | | Use Adafruit Motor Shield v2 connected by I2C instead of simple TB6612 or L298 breakout board.<br/>
This disables tone output by using motor as loudspeaker, but requires only 2 I2C/TWI pins in contrast to the 6 pins used for the full bride.<br/>
For full bride, analogWrite the millis() timer0 is used since we use pin 5 & 6. |
| `USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD` |  enabled | PWMDcMotor.h | Saves 694 bytes program memory  |
| `SUPPORT_RAMP_UP` | enabled | PWMDcMotor.h | `DO_NOT_SUPPORT_RAMP_UP` | Saves around 300 bytes program memory |
|-|-|-|-|
| `USE_LAYOUT_FOR_NANO` | disabled | RobotCar.h | Use different pinout for Nano board. It has A6 and A7 available as pins. |
| `CAR_HAS_4_WHEELS` | disabled | RobotCar.h | Use modified formula dor turning the car. |
| `USE_US_SENSOR_1_PIN_MODE` | disabled | RobotCar.h | Use modified HC-SR04 modules or HY-SRF05 ones.</br>Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger pin. |
| `CAR_HAS_IR_DISTANCE_SENSOR` | disabled | RobotCar.h | Use Sharp GP2Y0A21YK / 1080 IR distance sensor. |
| `CAR_HAS_TOF_DISTANCE_SENSOR` | disabled | RobotCar.h | Use VL53L1X TimeOfFlight distance sensor. |
| `DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN` | disabled | Distance.h | The distance servo is mounted head down to detect even small obstacles. |
| `CAR_HAS_CAMERA` | disabled | RobotCar.h | Enables the `Camera` button for the `PIN_CAMERA_SUPPLY_CONTROL` pin. |
| `CAR_HAS_LASER` | disabled | RobotCar.h | Enables the `Laser` button for the `PIN_LASER_OUT` / `LED_BUILTIN` pin. |
| `CAR_HAS_PAN_SERVO` | disabled | RobotCar.h | Enables the pan slider for the `PanServo` at the `PIN_PAN_SERVO` pin. |
| `CAR_HAS_TILT_SERVO` | disabled | RobotCar.h | Enables the tilt slider for the `TiltServo` at the `PIN_TILT_SERVO` pin.. |
| `MONITOR_LIPO_VOLTAGE` | disabled | RobotCar.h | Shows VIN voltage and monitors it for undervoltage. Requires 2 additional resistors at pin A2. |
| `VIN_VOLTAGE_CORRECTION` | undefined | RobotCar.h | Voltage to be substracted from VIN voltage. E.g. if there is a series diode between LIPO and VIN set it to 0.8. |



# Wall detection
The problem of the ultrasonic values is, that you can only detect a wall with the ultrasonic sensor if the angle of the wall relative to sensor axis is approximately between 70 and 110 degree.
For other angels the reflected ultrasonic beam can not not reach the receiver which leads to unrealistic great distances.<br/>
Therefore I take samples every 18 degrees and if I get 2 adjacent short (<DISTANCE_FOR_WALL_DETECT) distances, I assume a wall determined by these 2 samples.
The (invalid) values 18 degrees right and left of these samples are then extrapolated by computeNeigbourValue().

# Pictures
2 wheel car
![2 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/2WheelDriveCar.jpg)
4 wheel car
![4 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/4WheelDriveCar.jpg)
Encoder fork sensor
![Encoder fork sensor](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ForkSensor.jpg)
Servo mounting
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


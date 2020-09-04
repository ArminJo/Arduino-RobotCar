# Autonomous driving robot car

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://github.com/ArminJo/Arduino-RobotCar/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Arduino-RobotCar/actions)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FArduino-RobotCar)](https://github.com/brentvollebregt/hit-counter)

Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the environment.
Manual control is implemented by a GUI using a Bluetooth HC-05 Module and the BlueDisplay library.

Just overwrite the function doUserCollisionDetection() to test your own skill.

You may also overwrite the function fillAndShowForwardDistancesInfo(), if you use your own scanning method.

## Wall detection
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


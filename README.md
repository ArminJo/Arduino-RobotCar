# Autonomous driving robot car
Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the area.
Manual control is by a GUI implemented with a Bluetooth HC-05 Module and the BlueDisplay library.

Just overwrite the 2 functions fillForwardDistancesInfoSimple() and doCollisionDetectionSimple() to test your own skill.

#Pictures
2 wheel car
![2 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/media/2WheelDriveCar.jpg)
4 wheel car
![4 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/media/4WheelDriveCar.jpg)
Encoder fork sensor
![Encoder fork sensor](https://github.com/ArminJo/Arduino-RobotCar/blob/master/media/ForkSensor.jpg)
Servo mounting
![Servo mounting](https://github.com/ArminJo/Arduino-RobotCar/blob/master/media/ServoAtTopBack.jpg)
VIN sensing
![VIN sensing](https://github.com/ArminJo/Arduino-RobotCar/blob/master/media/SensingVIn.jpg)

#SCREENSHOTS
Manual control page
![Manual control page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/media/ManualControl.png)
Automatic control page with detected wall at right
![Automatic control page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/media/AutoControlWithWallDetected.png)


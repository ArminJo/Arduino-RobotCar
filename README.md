
<div align = center>

# [Autonomous driving robot car](https://github.com/ArminJo/Arduino-RobotCar)
Enables autonomous driving of a 2 or 4 wheel or mecanum wheel car controlled by an Arduino.<br/>
The car can be remotely controlled using the BlueDisplay library and the Android app.

[![Badge License: GPLv3](https://img.shields.io/badge/License-GPLv3-brightgreen.svg)](https://www.gnu.org/licenses/gpl-3.0)
 &nbsp; &nbsp;
[![Badge Version](https://img.shields.io/github/v/release/ArminJo/Arduino-RobotCar?color=yellow&logo=DocuSign&logoColor=white)](https://github.com/ArminJo/Arduino-RobotCar/releases/latest)
 &nbsp; &nbsp;
[![Badge Commits since latest](https://img.shields.io/github/commits-since/ArminJo/Arduino-RobotCar/latest?color=yellow)](https://github.com/ArminJo/Arduino-RobotCar/commits/master)
 &nbsp; &nbsp;
[![Badge Build Status](https://github.com/ArminJo/Arduino-RobotCar/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Arduino-RobotCar/actions)
 &nbsp; &nbsp;
![Badge Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_Arduino-RobotCar)
<br/>
<br/>
[![Stand With Ukraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://stand-with-ukraine.pp.ua)

Used as [RobotCarBlueDisplay](https://github.com/ArminJo/PWMMotorControl/tree/master/examples/RobotCarBlueDisplay) example of the Arduino library [PWMMotorControl](https://github.com/ArminJo/PWMMotorControl).<br/>
Requires the Arduino libraries [BlueDisplay](https://github.com/ArminJo/Arduino-BlueDisplay), [PWMMotorControl](https://github.com/ArminJo/PWMMotorControl) and [PlayRtttl](https://github.com/ArminJo/PlayRtttl).
</div>

#### If you find this program useful, please give it a star.

&#x1F30E; [Google Translate](https://translate.google.com/translate?sl=en&u=https://github.com/ArminJo/Arduino-RobotCar)

<br/>

# Features
- **Manual control by a GUI** on your Android smartphone or tablet using a Bluetooth HC-05 module and the [BlueDisplay app](https://play.google.com/store/apps/details?id=de.joachimsmeyer.android.bluedisplay).
- **Sensor drive** - control the car by tilting your smartphone or tablet.
- **Obstacle avoidance** with a HC-SR04 ultrasonic sensor mounted on a SG90 servo, which continuously scans the area in front of the car.
- To overcome the drawbacks of ultrasonic sensors, an additional **Sharp IR distance sensor** (types 430, 1080, 20150 or 100550) or a **VL53L1X TOF** (TimeOfFlight) sensor can be mounted on the servo.
- **Wall detection** to compensate the weakness of ultrasonic sensors for walls with a flat angle.
- **Distance follower** - the car follows a target (e.g. your hand) and keeps a distance of 22 to 30 cm.
- **Line follower** with a TCRT5000 3-channel IR sensor. Supports black or white lines and stop marks.
- **Demo mode** if no Bluetooth connection is established within 30 seconds after boot.
- **Path info page** shows the path driven in obstacle avoidance mode (requires encoders).
- **Calibration** of drive speed and rotation, values are stored in EEPROM.
- **VIN voltage monitoring** with undervoltage detection (requires a voltage divider).
- Plays **RTTTL melodies** at the end of the initial timeout and by GUI button.
- Support for **many predefined car configurations**, e.g. for **2, 4 or mecanum wheel cars**, different **motor drivers** (L298, TB6612, Adafruit Motor Shield V2), different **power supplies** and different **distance sensors**.

<br/>

# Installation
### Required libraries
Install the following libraries with *Tools > Manage Libraries...* or *Ctrl+Shift+I*:
- **[BlueDisplay](https://github.com/ArminJo/Arduino-BlueDisplay)** - use "BlueDisplay" as filter string.
- **[PWMMotorControl](https://github.com/ArminJo/PWMMotorControl)** - use "PWMMotorControl" as filter string.
- **[PlayRtttl](https://github.com/ArminJo/PlayRtttl)** - use "PlayRtttl" as filter string. Not required if `NO_RTTTL_FOR_CAR` is defined.
- **Servo** - required for the distance servo, if `USE_LIGHTWEIGHT_SERVO_LIBRARY` is not defined. Bundled with the Arduino IDE.
- **Adafruit Motor Shield V2 Library** - only for configurations with the Adafruit Motor Shield.

### Get the sketch
- Open the example **[RobotCarBlueDisplay](https://github.com/ArminJo/PWMMotorControl/tree/master/examples/RobotCarBlueDisplay)**, available at *File > Examples > Examples from Custom Libraries / PWMMotorControl*.

OR

- Download the [Zip file](https://github.com/ArminJo/Arduino-RobotCar/archive/refs/heads/master.zip) of this repository with *Code > Download ZIP*.
Open it and extract the sub folder **RobotCarBlueDisplay** to the **Sketchbook location** you see in the preferences window *File > Preferences (Ctrl + Comma)*. After restarting the Arduino IDE, you can open it as RobotCarBlueDisplay with *File > Sketchbook*.

Other examples of PWMMotorControl like **[SmartCarFollower](https://github.com/ArminJo/PWMMotorControl#smartcarfollower)** will also run on this robot car.

<br/>

# Quick start - select your car configuration
All hardware related settings are bundled in predefined configurations in [RobotCarConfigurations.h](RobotCarBlueDisplay/RobotCarConfigurations.h).<br/>
**Select your configuration by enabling exactly one `*_CONFIGURATION` macro at the top of `RobotCarBlueDisplay.ino`**, before the line `#include "RobotCarConfigurations.h"`.
If no configuration is selected, `L298_2WD_4AA_BASIC_CONFIGURATION` is used.

The name of the active configuration is printed at startup on the serial output, unless `NO_SERIAL_OUTPUT` is defined.

All configurations except `MECANUM_BASIC_CONFIGURATION` contain a HC-SR04 ultrasonic distance sensor mounted on a servo.

| Configuration | Hardware |
|-|-|
| **L298 bridge** | |
| `L298_2WD_4AA_BASIC_CONFIGURATION` | **Default.** China 2WD set with L298 bridge, Uno with series diode for VIN and 4 AA batteries. |
| `L298_2WD_2LI_ION_BASIC_CONFIGURATION` | As above, but with 2 Li-ion cells. |
| `L298_2WD_2LI_ION_VIN_IR_CONFIGURATION` | 2 Li-ion basic + VIN voltage divider + IR distance sensor. |
| `L298_2WD_2LI_ION_VIN_IR_IMU_CONFIGURATION` | 2 Li-ion basic + VIN voltage divider + IR distance sensor + MPU6050. |
| `L298_4WD_4AA_BASIC_CONFIGURATION` | China ZK-4WD set with L298 bridge and 4 AA batteries. |
| **TB6612 bridge** | |
| `TBB6612_4WD_4AA_BASIC_CONFIGURATION` | China 4WD set with TB6612 mosfet bridge and 4 AA batteries. |
| `TBB6612_4WD_4AA_VIN_CONFIGURATION` | + VIN voltage divider. |
| `TBB6612_4WD_4AA_FULL_CONFIGURATION` | + VIN voltage divider + MPU6050. |
| `TBB6612_4WD_4NIMH_BASIC_CONFIGURATION` | With 4 NiMh batteries (4.8 volt). |
| `TBB6612_4WD_4NIMH_VIN_CONFIGURATION` | With 4 NiMh batteries + VIN voltage divider. |
| `TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION` | With 2 Li-ion cells connected to the Uno power jack. |
| `TBB6612_4WD_2LI_ION_FULL_CONFIGURATION` | With 2 Li-ion cells + VIN voltage divider + MPU6050. |
| `TBB6612_4WD_2LI_ION_DIRECT_BASIC_CONFIGURATION` | With 2 Li-ion cells connected directly to VIN. **See the hardware notes in RobotCarConfigurations.h!** |
| `TBB6612_4WD_2LI_ION_DIRECT_VIN_CONFIGURATION` | As above + VIN voltage divider. |
| **Adafruit Motor Shield V2** (2 Li-ion, VIN voltage divider, servo mounted head down) | |
| `MOTOR_SHIELD_2WD_BASIC_CONFIGURATION` | 2WD basic. |
| `MOTOR_SHIELD_2WD_ENCODER_CONFIGURATION` | + encoders. |
| `MOTOR_SHIELD_2WD_TOF_CONFIGURATION` | + VL53L1X TOF sensor. |
| `MOTOR_SHIELD_2WD_ENCODER_TOF_CONFIGURATION` | + encoders + VL53L1X TOF sensor. |
| `MOTOR_SHIELD_2WD_FULL_CONFIGURATION` | + encoders + VL53L1X TOF sensor + MPU6050. |
| `MOTOR_SHIELD_4WD_BASIC_CONFIGURATION` | 4WD basic. |
| `MOTOR_SHIELD_4WD_FULL_CONFIGURATION` | 4WD + encoders + IR distance sensor. |
| `MOTOR_SHIELD_4WD_FULL_AND_LINE_FOLLOWER_CONFIGURATION` | 4WD + encoders + IR distance sensor + TCRT5000 line follower sensor at the back of the car. |
| **Arduino Nano on breadboard** | |
| `BREADBOARD_4WD_FULL_CONFIGURATION` | TB6612, 2 Li-ion, pan/tilt servos, MPU6050, camera and laser. |
| `MECANUM_BASIC_CONFIGURATION` | TB6612, 2 Li-ion, 4 mecanum wheels, **no distance sensor**. |
| `MECANUM_US_DISTANCE_CONFIGURATION` | Mecanum + ultrasonic sensor on a servo mounted head down. |
| **Other** | |
| `CAR_IS_ESP32_CAM_BASED` | Car controlled by an ESP32-CAM, like the [ESP32-Cam-Sewer-inspection-car](https://github.com/ArminJo/ESP32-Cam-Sewer-inspection-car). |

If no configuration fits your car, start with the most similar one and modify the [hardware macros](#hardware-macros) in RobotCarConfigurations.h.

<br/>

# Driving modes
The modes are selected on the *Automatic control* page, which is available if the car has a distance sensor on a servo.

### Obstacle avoidance
The car drives forward while scanning 10 directions from 9 to 171 degree in steps of 18 degree. In single step mode, it drives 20 cm per scan.
After each scan, the built-in function `doBuiltInCollisionAvoiding()` decides the next rotation.
If all distances are too short, the car goes backwards and scans again.

### Distance follower
The car scans at 70, 90 and 110 degree.
- If a target is found straight ahead, the car keeps a distance between 22 and 30 cm by driving forward or backward with a speed proportional to the difference.
- If the target is only found at one side (up to 70 cm), the car turns towards it.
- Targets farther away than 70 cm are ignored.

The distances can be changed in [AutonomousDrive.h](RobotCarBlueDisplay/AutonomousDrive.h).

### Line follower
Requires `CAR_HAS_LINE_FOLLOWER_SENSOR`, i.e. a TCRT5000 3-channel IR track sensor.
- Follows a black line by default. If `LINE_FOLLOWER_ENABLE_WHITE_LINE_PIN` is connected to ground, a white line is followed.
- A bar across the track, which covers all 3 sensors, is a stop mark.
- Define `LINE_FOLLOWER_SENSORS_ARE_MOUNTED_AT_BACK_OF_CAR` if the sensor is mounted at the back of the car.

### Step modes
The step mode button cycles through:
- **Continuous** - the car drives without stopping.
- **Step to turn** - the car stops before each turn; the next step is started with the *Step* button. Treated as continuous for the line follower.
- **Single step** - the car does one step and then stops.

### Distance source modes
If the car has an IR or TOF sensor **in addition** to the ultrasonic sensor, the button at the bottom right selects which value is used:
**Minimum** of both, **Maximum** of both, **US** only, or **IR / TOF** only (default).<br/>
If `US_DISTANCE_SENSOR_ENABLE_PIN` is connected to ground at startup, the US sensor is used.

<br/>

# Behavior without Bluetooth connection
If no Bluetooth connection is established **30 seconds** after boot and the car is **not USB powered**, a melody is played and the demo starts.
The demo runs until a connection is established or undervoltage is detected.

| Car | `AUX_PIN` high or not connected | `AUX_PIN` connected to ground |
|-|-|-|
| Mecanum car | All demo moves in a row every 2 minutes | Short demo every minute |
| Car with line follower sensor | Line follower mode for 2 minutes | Simple demo (forward, backward, turns) every minute |
| Car with distance sensor on servo | Distance follower mode | Simple demo every minute |
| Other | Simple demo every minute | Simple demo every minute |

`AUX_PIN` is defined only for `MECANUM_BASIC_CONFIGURATION` (pin A3) and `MOTOR_SHIELD_4WD_BASIC_CONFIGURATION` (pin 9). Without `AUX_PIN`, the left column applies.

USB power can only be detected if the car has a VIN voltage divider or if VCC can be measured internally.

**With a connection**, the car plays a double tone and scans with the distance servo after 4 minutes without user input, and then every 2 minutes.

<br/>

# Pictures
| 4WD car with IR receiver and Bluetooth module and 4 AA rechargeable batteries. | Instructable |
|-|-|
| ![4 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/4WDInstructable.jpg) | [![Instructable](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/instructables-logo-v2.png)](https://www.instructables.com/Arduino-4WD-Car-Assembly-and-Code-With-Optional-In/) |
| 4 wheel car like the 2 WD car right, but with servo mounted head up. | 2 wheel car with encoders, 2 Li-ion batteries, Adafruit Motor Shield V2, Bluetooth connection, and servo mounted head down.  |
| ![4 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/4WheelDriveCar.jpg) | ![2 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/2WheelDriveCar.jpg)  |
| Encoder fork sensor | Servo mounted head down |
| ![Encoder fork sensor](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ForkSensor.jpg) | ![Servo mounting](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ServoAtTopBack.jpg) |
| VIN sensing with motor shield | Red car with Sharp GP2Y0A21YK distance sensor |
| ![VIN sensing](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/SensingVIn.jpg) | ![Red car with Sharp GP2Y0A21YK distance sensor](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/RedCar.jpg)  |
| **Youtube video** of MecanumWheelCar demo | **Youtube video** of mecanum car controlled by the accelerator sensors of the tablet |
| [![MecanumWheelCar Demo](https://i.ytimg.com/vi/VX92jeqr8_g/hqdefault.jpg)](https://www.youtube.com/watch?v=VX92jeqr8_g) | [![Arduino Mecanum Wheel Car Sensor Drive Demo](https://i.ytimg.com/vi/lhlxLTj9Mrg/hqdefault.jpg)](https://www.youtube.com/watch?v=lhlxLTj9Mrg) |

<br/>

# GUI pages
| Start page | Test page |
|-|-|
| ![Start page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/HomePage.png) | ![Test page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/TestPage.png) |
| Sensor drive page | Sensor drive page for mecanum car |
| ![Sensor drive page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/SensorDrivePage.jpg) | ![Sensor drive page for mecanum car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/SensorDrivePageWithTurn.jpg) |

- **Start page** - manual control with speed sliders, distance servo position and distance bars. The *CAL* button (also on the test page) starts the calibration of drive speed and rotation.
- **Sensor drive page** - drive by tilting the smartphone or tablet.
- **Automatic control page** - obstacle avoidance, distance follower and line follower.
- **Test page** - test of motors, distances and rotations.
- **Path info page** - path driven in obstacle avoidance mode. Requires encoders and is disabled by `NO_PATH_INFO_PAGE`.

### Automatic control page with detected wall at right
![Automatic control page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/AutoDrivePageWallDetected.jpg)
- **Cyan** bars are distances of 110 cm and more (timeout).
- **Green** bars are distances greater than double the distance of one ride per scan (default 40 cm).
- **Yellow** bars are the values between the 2 thresholds (default 20 cm to 40 cm).
- **Red** bars are distances below the distance of one ride per scan (default 20 cm) -> collision during next "scan and ride" cycle if obstacle is ahead.
- The tiny **white** bars are the distances computed by the `doWallDetection()` function. They overlay the green (assumed timeout) values.
- The **blue** line is the rotation chosen by the `doBuiltInCollisionAvoiding()` function. It is **red** if the car has to turn around.

<br/>

# Wall detection
Ultrasonic distance measurement has a problem with walls.
You can **only detect a wall** if the angle of the wall relative to sensor axis is approximately **between 70 and 110 degree**.
For other angles the reflected ultrasonic beam cannot reach the receiver which leads to unrealistic great distances.<br/>
The implemented wall detection function `doWallDetection()` takes samples every 18 degrees and if it gets 2 adjacent short distances below `DISTANCE_MAX_FOR_WALL_DETECTION_CM` (40 cm), it assumes a wall determined by these 2 samples.
The (invalid) values 18 degrees right and left of these samples are then extrapolated by `computeNeigbourValue()`.

Wall detection is only used if the US sensor value is taken, i.e. not in distance source modes *Minimum* and *IR / TOF*.

<br/>

# Pin layout
The complete pin mapping is documented in [RobotCarPinDefinitionsAndMore.h](RobotCarBlueDisplay/RobotCarPinDefinitionsAndMore.h).

| Function | Uno with L298 or TB6612 | Uno with Adafruit Motor Shield | Nano (breadboard) |
|-|-|-|-|
| Motors | 4, 5, 6, 7, 8, 9 | I2C (A4, A5) | 4 to 9, 4 to 12 for mecanum car |
| Encoders right / left | 2 / 3 | 2 / 3 | 2 / 3 |
| Distance servo | 10 | 10 | 10, 13 for mecanum car |
| US trigger / echo | A0 / A1 | A0 / A1 | A0 / A1 |
| IR distance sensor | A3 | A3 | A6 |
| VIN voltage divider (VIN/11) | A2 | A2 | A7 |
| Buzzer | 12 | 12 | A3, A5 for mecanum car |
| I2C (TOF sensor, MPU6050) | A4, A5 | A4, A5 | A4, A5 |
| Line follower sensor left / mid / right | 2 / 3 / 11 | 4 / 5 / 6 with encoders | - |
| `LINE_FOLLOWER_ENABLE_WHITE_LINE_PIN` | - | 8 | - |
| `US_DISTANCE_SENSOR_ENABLE_PIN` | 2 if no encoders | 7 with encoders | - |
| `AUX_PIN` | - | 9 for 4WD | A3 for mecanum car |

Input pins like `AUX_PIN` or `US_DISTANCE_SENSOR_ENABLE_PIN` have the internal pullup enabled. Connect them to ground to activate the function.

<br/>

# Compile options / macros for this software
To customize the software to different car configurations, there are some compile options / macros available.<br/>
**Normally you only have to select a [configuration](#quick-start---select-your-car-configuration).**
The macros below are set by the configurations, or can be added to the configuration in RobotCarConfigurations.h or at the top of RobotCarBlueDisplay.ino, **before** `#include "RobotCarConfigurations.h"`.

Compile options for the used **PWMMotorControl library** like `USE_ENCODER_MOTOR_CONTROL`, `USE_ADAFRUIT_MOTOR_SHIELD`, `USE_L298_BRIDGE` or `VIN_2_LI_ION` are described [here](https://github.com/ArminJo/PWMMotorControl#compile-options--macros-for-this-library).

### Hardware macros
These are defined in RobotCarConfigurations.h. *Default* is the value for the default configuration `L298_2WD_4AA_BASIC_CONFIGURATION`.

| Name | Default | Description |
|-|-|-|
| `CAR_HAS_US_DISTANCE_SENSOR` | defined | A HC-SR04 ultrasonic distance sensor is mounted. |
| `US_SENSOR_SUPPORTS_1_PIN_MODE` | undefined | Use modified HC-SR04 modules or HY-SRF05 ones.<br/>Modify HC-SR04 by connecting 10 k&ohm; between echo and trigger and then use only the trigger pin. |
| `CAR_HAS_IR_DISTANCE_SENSOR` | undefined | A Sharp IR distance sensor is mounted. Select the type with `IR_SENSOR_TYPE_*`. |
| `IR_SENSOR_TYPE_430`<br/>`IR_SENSOR_TYPE_1080`<br/>`IR_SENSOR_TYPE_20150`<br/>`IR_SENSOR_TYPE_100550` | `IR_SENSOR_TYPE_1080` | Sharp sensor type:<br/>GP2YA41SK0F 4 to 30 cm,<br/>GP2Y0A21YK0F 10 to 80 cm,<br/>GP2Y0A02YK0F 20 to 150 cm,<br/>GP2Y0A710K0F 100 to 550 cm. |
| `CAR_HAS_TOF_DISTANCE_SENSOR` | undefined | A VL53L1X TimeOfFlight distance sensor is mounted. |
| `CAR_HAS_DISTANCE_SERVO` | defined | The distance sensor is mounted on a servo. 0 degree is right, 180 degree is left. |
| `DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN` | undefined | The distance servo is mounted head down to detect even small obstacles. The servo direction is reversed then. |
| `CAR_HAS_LINE_FOLLOWER_SENSOR` | undefined | A TCRT5000 3-channel IR track sensor is mounted. |
| `LINE_FOLLOWER_SENSORS_ARE_MOUNTED_AT_BACK_OF_CAR` | undefined | Switches forward and backward directions for the line follower. |
| `CAR_HAS_ENCODERS` | undefined | 2 slot-type photo interrupters are connected to pin 2 and 3. Enables `USE_ENCODER_MOTOR_CONTROL`. |
| `CAR_HAS_MPU6050_IMU` | undefined | A MPU6050 breakout board like GY-521 is connected by I2C for precise turning. Enables `USE_MPU6050_IMU`. |
| `CAR_HAS_VIN_VOLTAGE_DIVIDER` | undefined | VIN/11 at A2 (A7 on Nano), e.g. 1 M&ohm; to VIN, 100 k&ohm; to ground. Required to show and monitor (for undervoltage) VIN voltage. |
| `VIN_VOLTAGE_CORRECTION` | 0.81 | Voltage to be added to the measured VIN voltage to compensate the series diode between power jack and VIN on the Uno board. |
| `CAR_HAS_4_WHEELS` | undefined | The car has 4 wheels. Affects rotation calibration. |
| `CAR_HAS_4_MECANUM_WHEELS` | undefined | The car has 4 mecanum wheels. Implies `CAR_HAS_4_WHEELS` and a different pin layout. |
| `CAR_HAS_PAN_SERVO` | undefined | Enables the pan slider for the `PanServo` at the `PAN_SERVO_PIN` pin. |
| `CAR_HAS_TILT_SERVO` | undefined | Enables the tilt slider for the `TiltServo` at the `TILT_SERVO_PIN` pin. |
| `CAR_HAS_CAMERA` | undefined | Enables the `Camera` button for the `CAMERA_SUPPLY_CONTROL_PIN` pin. |
| `CAR_HAS_LASER` | undefined | Enables the `Laser` button for the `LASER_OUT_PIN` / `LED_BUILTIN` pin. |
| `CAR_IS_NANO_BASED` | undefined | An Arduino Nano is used instead of an Uno, resulting in a different pin layout. |
| `AUX_PIN` | undefined | Input pin to select the demo mode. See [Behavior without Bluetooth connection](#behavior-without-bluetooth-connection). |

### Software macros
| Name | Default | Description |
|-|-|-|
| `NO_PATH_INFO_PAGE` | defined | Disables the path info page. Saves up to 1400 bytes program memory. The page requires encoders anyway. |
| `NO_SERIAL_OUTPUT` | undefined | Disables all serial output. Saves up to 2886 bytes program memory. |
| `NO_RTTTL_FOR_CAR` | undefined | Disables the melodies and the *Melody* button. Saves up to 3654 bytes program memory. If not defined, `ENABLE_RTTTL_FOR_CAR` is defined. |
| `USE_MOTOR_FOR_MELODY` | undefined | Generates the melody tones with the coils of the left motor. Not for the motor shield. |
| `USE_LIGHTWEIGHT_SERVO_LIBRARY` | undefined | Use the LightweightServo library for servos at pin 9 and 10 instead of the Arduino Servo library. Saves up to 710 bytes program memory. |
| `BLUETOOTH_BAUD_RATE` | `BAUD_9600` | Set to `BAUD_115200` if you have reprogrammed your HC-05 module. |
| `DEFAULT_CIRCUMFERENCE_MILLIMETER` | 220 | Circumference of your wheels in millimeter. Required for exact distances. |
| `DISTANCE_SERVO_TRIM_DEGREE` | undefined | This value is added to all distance servo positions to compensate mounting tolerances. |
| `TOF_OFFSET_MILLIMETER` | undefined | Offset for the VL53L1X sensor. Offset = RealDistance - MeasuredDistance. |
| `DISTANCE_SOURCE_MODE_DEFAULT` | `DISTANCE_SOURCE_MODE_IR_OR_TOF` | Initial [distance source mode](#distance-source-modes) if 2 distance sensors are mounted. |
| `FOLLOWER_DISTANCE_MINIMUM_OK_CENTIMETER` | 22 | Distance follower goes backwards below this distance. Defined in AutonomousDrive.h. |
| `FOLLOWER_DISTANCE_MAXIMUM_OK_CENTIMETER` | 30 | Distance follower goes forward above this distance. Defined in AutonomousDrive.h. |
| `FOLLOWER_DISTANCE_MAXIMUM_DISCOVERY_CENTIMETER` | 70 | Distance follower ignores targets farther away. Defined in AutonomousDrive.h. |

# Revision history
### Version 3.1
- Added line follower.

### Version 3.0
- Compatibility to latest BlueDisplay version.

### Version 2.0
- Improvements and renaming.

### Version 1.0
- Initial version.

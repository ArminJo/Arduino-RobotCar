/*
 * RobotCarMotorControl.h
 *
 *  Created on: 29.09.2016
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef SRC_ROBOTCAR_H_
#define SRC_ROBOTCAR_H_

#include <Arduino.h>
#include <Servo.h>

// Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger.
//#define USE_US_SENSOR_1_PIN_MODE // Comment it out, if you use modified HC-SR04 modules or HY-SRF05 ones.

//#define CAR_HAS_IR_DISTANCE_SENSOR

//#define CAR_HAS_TOF_DISTANCE_SENSOR

//#define CAR_HAS_CAMERA

//#define CAR_HAS_LASER

/*
 * For pan tilt we have 2 servos in total
 */
//#define CAR_HAS_PAN_SERVO
//#define CAR_HAS_TILT_SERVO
/*
 * Use simple TB6612 breakout board instead of adafruit motor shield.
 * This enables tone output by using motor as loudspeaker, but needs 6 pins in contrast to the 2 TWI pins used for the shield.
 * For analogWrite the millis() timer0 is used since we use pin 5 & 6.
 */
//#define USE_TB6612_BREAKOUT_BOARD
/*
 * Plays melody after initial timeout has reached
 * Enables the Play Melody button
 */
// #define ENABLE_RTTTL
//
#ifdef USE_TB6612_BREAKOUT_BOARD
// Attached modules on the Nano version with breakout board
#define CAR_HAS_PAN_SERVO
#define CAR_HAS_TILT_SERVO
#define CAR_HAS_CAMERA
#define CAR_HAS_LASER
#define ENABLE_RTTTL
#endif

#include "CarMotorControl.h"
#include "AutonomousDrive.h"

#define CENTIMETER_PER_RIDE 20

#define MINIMUM_DISTANCE_TO_SIDE 21
#define MINIMUM_DISTANCE_TO_FRONT 35

/*
 * Pin usage
 * First the function the nano board variant. For this variant the PWM is generated with analogWrite()
 */
/*
 * PIN  I/O Function
 *   2  I   Right motor encoder
 *   3  I   Left motor encoder
 *   4  O   Motor 0 fwd / NC for UNO board
 *   5  O   Motor 0 PWM / NC for UNO board
 *   6  O   Motor 1 PWM / NC for UNO board
 *   7  O   Motor 0 back / NC for UNO board
 *   8  O   Motor 1 fwd / NC for UNO board
 *   9  O   Servo US distance
 *   10 O   Servo laser pan
 *   11 O   Servo laser tilt / Speaker for UNO board
 *   12 O   Motor 1 back / Two wheel detection / Input Pullup for UNO board
 *   13 O   Laser power
 *
 *   A0 I   VIN/11, 1MOhm to VIN, 100kOhm to ground.
 *   A1 O   US trigger
 *   A2 I   IR distance (needs 1 pin US sensor mode) / US echo
 *   A3 IP  Two wheel detection with input pullup
 *   A4 SDA NC for Nano / I2C for UNO board motor shield
 *   A5 SCL NC for Nano / I2C for UNO board motor shield
 *   A6 O   Speaker for Nano / not available on UNO board
 *   A7 O   Camera supply control
 */

/*
 * Motor control by TB6612 breakout board
 */

/*
 * Pins 9 + 10 are used for Servo library
 * 2 + 3 are used for encoder input
 */

#define PIN_LEFT_MOTOR_FORWARD     4
#define PIN_LEFT_MOTOR_BACKWARD    7
#define PIN_LEFT_MOTOR_PWM         5 // PWM capable

#define PIN_RIGHT_MOTOR_FORWARD     8
#define PIN_RIGHT_MOTOR_BACKWARD   12
#define PIN_RIGHT_MOTOR_PWM         6 // PWM capable

/*
 * Servos
 */
#define PIN_DISTANCE_SERVO       9
#ifdef CAR_HAS_PAN_SERVO
#define PIN_LASER_SERVO_PAN     10
extern Servo LaserPanServo;
#endif
#ifdef CAR_HAS_TILT_SERVO
#define PIN_LASER_SERVO_TILT    11
#define TILT_SERVO_MIN_VALUE     7 // since lower values will make an insane sound at my pan tilt device
extern Servo TiltServo;
#endif

/*
 * Distance sensors
 */
#define PIN_TRIGGER_OUT         A1
#ifndef USE_US_SENSOR_1_PIN_MODE
#define PIN_ECHO_IN             A2
#endif

#define DISTANCE_TIMEOUT_CM 100 // do not measure and process distances greater than 100 cm
#define DISTANCE_TIMEOUT_COLOR COLOR_CYAN

/*
 * Timeouts
 */
#define TIMOUT_AFTER_LAST_BD_COMMAND_MILLIS 240000L // move Servo after 4 Minutes of inactivity
#define TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS 10000 // Start demo mode 10 seconds after boot up

/*
 * Pin and ADC channel assignments
 */
// if connected to ground we have a 2 WD CAR
#define PIN_TWO_WD_DETECTION    A3

#define VIN_11TH_IN_CHANNEL      0 // = A0
/*
 * Pin assignments which are different in breakout and UNO version
 */
// assume resistor network of 100k / 10k (divider by 11)
#ifdef USE_TB6612_BREAKOUT_BOARD
#define PIN_CAMERA_SUPPLY_CONTROL A7
#define PIN_SPEAKER               A6

#else // USE_TB6612_BREAKOUT_BOARD
#  ifdef USE_US_SENSOR_1_PIN_MODE
// Otherwise available as US echo pin
#define PIN_IR_DISTANCE_SENSOR  A2
#  endif
#define PIN_SPEAKER             11
#endif // USE_TB6612_BREAKOUT_BOARD

#ifdef CAR_HAS_LASER
#define PIN_LASER_OUT           13
#endif

extern CarMotorControl RobotCarMotorControl;
extern float sVINVoltage;
#define VOLTAGE_LOW_THRESHOLD 6.9 // Formula: 2 * 3.5 volt - voltage loss: 25 mV GND + 45 mV VIN + 35 mV Battery holder internal
#define VOLTAGE_USB_THRESHOLD 5.5
#ifdef USE_TB6612_BREAKOUT_BOARD
#define VOLTAGE_CORRECTION 0.8 // For serial diode (needs 0.8 volt) between LIPO and VIN
#endif
#define VOLTAGE_TOO_LOW_DELAY_ONLINE 3000 // display VIN every 500 ms for 4 seconds
#define VOLTAGE_TOO_LOW_DELAY_OFFLINE 1000 // wait for 1 seconds after double beep
void readVINVoltage();

#ifdef ENABLE_RTTTL
extern bool sPlayMelody;
#endif

int doUserCollisionDetection();

#endif /* SRC_ROBOTCAR_H_ */

#pragma once

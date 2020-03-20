/*
 * RobotCarMotorControl.h
 *
 *  Created on: 29.09.2016
 *      Author: Armin
 */

#ifndef SRC_ROBOTCAR_H_
#define SRC_ROBOTCAR_H_

//#define USE_TB6612_BREAKOUT_BOARD

#include <Arduino.h>
#include <Servo.h>
#include "CarMotorControl.h"

#include "AutonomousDrive.h"

// Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger.
//#define USE_US_SENSOR_1_PIN_MODE // Comment it out, if you use modified HC-SR04 modules or HY-SRF05 ones.

//#define CAR_HAS_IR_DISTANCE_SENSOR

//#define CAR_HAS_TOF_DISTANCE_SENSOR

/*
 * For pan tilt we have 3 servos in total
 */
//#define USE_PAN_TILT_SERVO
/*
 * Use simple TB6612 breakout board instead of adafruit motor shield.
 * This enables tone output by using motor as loudspeaker, but needs 6 pins in contrast to the 2 TWI pins used for the shield.
 * For analogWrite the millis() timer0 is used since we use pin 5 & 6.
 */
//#define USE_TB6612_BREAKOUT_BOARD
#define CENTIMETER_PER_RIDE 20

#define MINIMUM_DISTANCE_TO_SIDE 21
#define MINIMUM_DISTANCE_TO_FRONT 35

/*
 * Pin usage
 * First the pins of the nano board variant. For this variant the PWM is generated with analogWrite()
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
 *   A0 I   VIN/11 for UNO board. 100kOhm to VIN, 10kOhm to ground.
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

#define PIN_MOTOR_0_FORWARD     4
#define PIN_MOTOR_0_BACKWARD    7
#define PIN_MOTOR_0_PWM         5 // PWM capable

#define PIN_MOTOR_1_FORWARD     8
#define PIN_MOTOR_1_BACKWARD   12
#define PIN_MOTOR_1_PWM         6 // PWM capable

/*
 * Servos
 */
#define PIN_DISTANCE_SERVO       9
#define PIN_LASER_SERVO_PAN     10
#define PIN_LASER_SERVO_TILT    11
extern Servo LaserPanServo;
#ifdef USE_PAN_TILT_SERVO
#define HAS_LASER
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

#ifdef HAS_LASER
#define PIN_LASER_OUT           13
#endif

extern CarMotorControl RobotCarMotorControl;
extern float sVINVoltage;
#define VOLTAGE_LOW_THRESHOLD 7.0
#define VOLTAGE_USB_THRESHOLD 5.5
void readVINVoltage();

#ifdef ENABLE_RTTTL
extern bool sPlayMelody;
#endif

int doUserCollisionDetection();

#endif /* SRC_ROBOTCAR_H_ */

#pragma once

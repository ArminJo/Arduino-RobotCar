/*
 * RobotCar.h
 *
 *  Created on: 29.09.2016
 *      Author: Armin
 */

#ifndef SRC_ROBOTCAR_H_
#define SRC_ROBOTCAR_H_

#include <Arduino.h>
#include <CarMotorControl.h>
#include <Servo.h>

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
 *   2  I   Right encoder
 *   3  I   Left encoder
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
 *   A0 O/I Camera supply control / VIN/11 for UNO board. 1MOhm to VIN, 100kOhm to ground.
 *   A1 O   US trigger
 *   A2 I   IR distance (needs 1 pin US sensor mode) / US echo
 *   A3 IP  Two wheel detection with input pullup
 *   A4 SDA NC for Nano / I2C for UNO board motor shield
 *   A5 SCL NC for Nano / I2C for UNO board motor shield
 *   A6 I   NC for Nano / not available on UNO board
 *   A7 I   VIN/11 for Nano board / not available on UNO board
 */

/*
 * Servos
 */
#define DISTANCE_SERVO_PIN        9
#define LASER_SERVO_PAN_PIN     10
#define LASER_SERVO_TILT_PIN    11
extern Servo LaserPanServo;
#ifdef USE_PAN_TILT_SERVO
#define HAS_LASER
#define TILT_SERVO_MIN_VALUE    7 // since lower values will make an insane sound at my pan tilt device
extern Servo TiltServo;
#endif

/*
 * Distance sensors
 */
#define TRIGGER_OUT_PIN         A1
#ifndef USE_US_SENSOR_1_PIN_MODE
#define ECHO_IN_PIN             A2
#endif


#define DISTANCE_TIMEOUT_CM 100 // do not measure and process distances greater than 100 cm
#define DISTANCE_TIMEOUT_COLOR COLOR_CYAN

/*
 * Pin assignments which are different in breakout and UNO version
 */
// assume resistor network of 100k / 10k (divider by 11)
#ifdef USE_TB6612_BREAKOUT_BOARD
#define CAMERA_SUPPLY_CONTROL_PIN A0
// if connected to ground we have a 2 WD CAR
#define TWO_WD_DETECTION_PIN    A3
#define VIN_11TH_IN_CHANNEL     7 // = A7 on Nano board
#else // USE_TB6612_BREAKOUT_BOARD
#  ifdef USE_US_SENSOR_1_PIN_MODE
// Now not available as echo pin
#define IR_DISTANCE_SENSOR_PIN  A2
#  endif
// if connected to ground we have a 2 WD CAR
#define TWO_WD_DETECTION_PIN    12
#define VIN_11TH_IN_CHANNEL      0 // = A0
#define SPEAKER_PIN             11
#endif // USE_TB6612_BREAKOUT_BOARD

#ifdef HAS_LASER
#define LASER_OUT_PIN           13
#endif

extern CarMotorControl RobotCar;
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

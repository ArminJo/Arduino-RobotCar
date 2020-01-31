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
 */
/*
 * PIN  I/O Function
 *   2  I   Right encoder
 *   3  I   Left encoder
 *   4  O   Motor 0 fwd
 *   5  O   Motor 0 PWM
 *   6  O   Motor 1 PWM
 *   7  O   Motor 0 back
 *   8  O   Motor 1 fwd
 *   9  O   Servo US
 *   10 O   Servo laser pan
 *   11 O   Servo tilt
 *   12 O   Motor 1 back
 *   13 O   Laser power
 *
 *   A0 I   VCC/11 for UNO board / Camera supply control for breakout board version
 *   A1 O   US trigger
 *   A2 I   US echo
 *   A3 I   Two wheel detection / Pullup
 *   A4 IO  I2C for Motor shield / SDA
 *   A5 O   I2C for Motor shield / SCL
 *   A6 I   Nano
 *   A7 I   VCC/11 for Nano board
 */

// if connected to ground we have a 2 WD CAR
const int TWO_WD_DETECTION_PIN = A3;


const uint8_t TRIGGER_OUT_PIN = A1;
const uint8_t ECHO_IN_PIN = A2;

// assume resistor network of 100k / 10k (divider by 11)
#ifdef USE_TB6612_BREAKOUT_BOARD
const uint8_t CAMERA_SUPPLY_CONTROL_PIN = A0;
const int VIN_11TH_IN_CHANNEL = 7; // = A7 on Nano board
#else
const int VIN_11TH_IN_CHANNEL = 0; // = A0
#endif

const int US_SERVO_PIN = 9;
const int LASER_SERVO_PAN_PIN = 10;
const int LASER_SERVO_TILT_PIN = 11;
extern Servo LaserPanServo;
#ifdef USE_PAN_TILT_SERVO
#define HAS_LASER
#define TILT_SERVO_MIN_VALUE 7 // since lower values will make an insane sound at my pan tilt device
extern Servo LaserTiltServo;
#endif

#ifdef HAS_LASER
const int LASER_OUT_PIN = 13;
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

/*
 * RobotCar.h
 *
 *  Created on: 29.09.2016
 *      Author: Armin
 */

#ifndef SRC_ROBOTCAR_H_
#define SRC_ROBOTCAR_H_

#include "AutonomousDrive.h"
#include <CarControl.h>

#define CENTIMETER_PER_RIDE 20

#define MINIMUM_DISTANCE_TO_SIDE 21
#define MINIMUM_DISTANCE_TO_FRONT 35

#define BREADBOARD_VERSION
/*
 * Pin usage
 */

// if connected to ground we have a 2 WD CAR
const int TWO_WD_DETECTION_PIN = 12;

// Must use digital pin 2 and 3 on Arduino, since they are connected to INT0 and INT1
const int LEFT_ENCODER_PIN = 2;
const int RIGHT_ENCODER_PIN = 3;
const int DEBUG_OUT_PIN = 5;

const uint8_t TRIGGER_OUT_PIN = A1;
const uint8_t ECHO_IN_PIN = A2;

// assume resistor network of 100k / 10k (divider by 11)
#ifdef BREADBOARD_VERSION
const int VCC_11TH_IN_CHANNEL = 7; // = A7 on Nano board
#else
#define USE_ADAFRUIT_MOTOR_SHIELD
const int VCC_11TH_IN_CHANNEL = 0; // = A0
#endif

// Used by the twi interface for the Adafruit motor shield
//const int SDA_PIN = A4;
//const int SCL_PIN = A5;

// determined by using LightweightServo lib
//const int US_SERVO_CONTROL_PIN = 9;
//const int LASER_SERVO_CONTROL_PIN = 10;

extern CarControl myCar;

bool myOwnFillForwardDistancesInfo(ForwardDistancesInfoStruct* aForwardDistancesInfo, bool aShowValues,
bool aDoFirstValue);
int myOwnDoCollisionDetection(ForwardDistancesInfoStruct* aForwardDistancesInfo);
#endif /* SRC_ROBOTCAR_H_ */

/*
 * RobotCar.h
 *
 *  Created on: 29.09.2016
 *      Author: Armin
 */

#ifndef SRC_ROBOTCAR_H_
#define SRC_ROBOTCAR_H_

#include <AutonomousDrive.h>
#include "CarControl.h"
//#include "Servo.h"

/*
 * Pin usage
 */
// Pin 13 has an LED connected on most Arduino boards.
const int LED_PIN = 13;

// if connected to ground we have a 2 WD CAR
const int TWOWD_DETECTION_PIN = 12;

// Must use digital pin 3 and 4 on Arduino, since they are connected to INT0 and INT1
const int DISTANCE_SENSOR_LEFT_PIN = 3;
const int DISTANCE_SENSOR_RIGHT_PIN = 4;
const int DEBUG1_PIN = 5;

extern uint8_t TRIGGER_OUT_PIN; // = A0;
extern uint8_t ECHO_IN_PIN; // = A1;

// assume resistor network of 100k / 10k (divider by 11)
const int VCC_11TH_IN_CHANNEL = 3; // = A3

//const int SDA_PIN = A4;
//const int SCL_PIN = A5;
const int SERVO_CURRENT_IN_CHANNEL = 2; // = A2
const int SERVO_CONTROL_PIN = 10;

extern CarControl myCar;

bool fillForwardDistancesInfoMyOwn(ForwardDistancesInfoStruct* aForwardDistancesInfo, bool aShowValues,
bool aDoFirstValue);
int doMyOwnCollisionDetection(ForwardDistancesInfoStruct* aForwardDistancesInfo);
#endif /* SRC_ROBOTCAR_H_ */

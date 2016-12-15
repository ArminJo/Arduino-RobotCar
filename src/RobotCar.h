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
#include "Servo.h"

extern CarControl myCar;
extern Servo sServoUS;

bool fillForwardDistancesInfoSimple(ForwardDistancesInfoStruct* aForwardDistancesInfo, Servo aServoForUS, bool aShowValues,
bool aDoFirstValue);
int doCollisionDetectionSimple(ForwardDistancesInfoStruct* aForwardDistancesInfo);
#endif /* SRC_ROBOTCAR_H_ */

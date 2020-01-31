/*
 * AutonomousDrive.h
 *
 *  Created on: 08.11.2016
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 */

#ifndef SRC_AUTONOMOUSDRIVE_H_
#define SRC_AUTONOMOUSDRIVE_H_

#include <Servo.h>
#include <stdint.h>

#define DEGREES_PER_STEP  18
#define STEPS_PER_SCAN    9 // -> 162 degrees for 18 DEGREES_PER_STEP
#define NUMBER_OF_DISTANCES (STEPS_PER_SCAN + 1)
#define START_DEGREES      ((180 - (DEGREES_PER_STEP * STEPS_PER_SCAN)) / 2) // 9 - we need it symmetrical in the 180 degrees range

#define INDEX_RIGHT 0
#define INDEX_LEFT STEPS_PER_SCAN
#if (STEPS_PER_SCAN == 9)
// Works only for STEPS_PER_SCAN = 9
#define INDEX_FORWARD_1 4
#define INDEX_FORWARD_2 5
#endif

struct ForwardDistancesInfoStruct {
    uint8_t RawDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degrees
    uint8_t ProcessedDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degrees
    uint8_t IndexOfMaxDistance;
    uint8_t IndexOfMinDistance;
    uint8_t MaxDistance;
    uint8_t MinDistance;
    // 0 degree => wall parallel to side of car. 90 degrees => wall in front of car. degrees of wall -> degrees to turn.
    int8_t WallRightAngleDegrees;
    int8_t WallLeftAngleDegrees;
};

extern ForwardDistancesInfoStruct sForwardDistancesInfo;

/*
 * Used for adaptive collision detection
 */
extern int sLastDecisionDegreesToTurnForDisplay;
extern int sNextDegreesToTurn;
extern int sLastDegreesTurned;

extern Servo USDistanceServo;
extern uint8_t sLastServoAngleInDegrees; // needed for optimized delay for servo repositioning

extern uint8_t sCentimeterPerScanTimesTwo;
extern uint8_t sCentimeterPerScan; // = sCentimeterPerScanTimesTwo / 2

void initUSServo();
void US_ServoWriteAndDelay(uint8_t aValue, bool doDelay = false);

/*
 * Values for included implementation
 */
const int CENTIMETER_PER_RIDE = 25;

// do not measure and process distances greater than 100 cm
#define US_TIMEOUT_CENTIMETER 100

// I measured ca. 110 ms
const int MILLIS_FOR_SERVO_20_DEGREES = 120;

void fillAndShowForwardDistancesInfo(bool aShowValues, bool aDoFirstValue);
void doWallDetection(bool aShowValues);
int doBuiltInCollisionDetection();
void driveAutonomousOneStep(int (*aCollisionDetectionFunction)());

#endif /* SRC_AUTONOMOUSDRIVE_H_ */

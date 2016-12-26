/*
 * AutonomousDrive.h
 *
 *  Created on: 08.11.2016
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 */

#ifndef SRC_AUTONOMOUSDRIVE_H_
#define SRC_AUTONOMOUSDRIVE_H_

//#include "Servo.h"
#include <stdint.h>

#define DEGREE_PER_STEP 20
#define STEPS_PER_180_DEGREE ((180 / DEGREE_PER_STEP))
#define NUMBER_OF_DISTANCES ((180 / DEGREE_PER_STEP) + 1)

#define INDEX_FORWARD_1 4
#define INDEX_FORWARD_2 5
#define INDEX_RIGHT 0
#define INDEX_LEFT STEPS_PER_180_DEGREE

struct ForwardDistancesInfoStruct {
    uint8_t RawDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degree
    uint8_t ProcessedDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degree
    uint8_t IndexOfMaxDistance;
    uint8_t IndexOfMinDistance;
    uint8_t MaxDistance;
    uint8_t MinDistance;
};

extern ForwardDistancesInfoStruct ForwardDistancesInfo;

/*
 * Used for adaptive collision detection
 */
extern int sLastDecisionDegreeToTurnForDisplay;
extern int sNextDegreeToTurn;
extern int sLastDegreeTurned;
extern uint8_t sLastServoAngleInDegree; // needed for optimized delay for servo repositioning

extern uint8_t sCountPerScan;
extern uint8_t sCentimeterPerScan; // = sCountPerScan / 2

void drawForwardDistancesInfos(ForwardDistancesInfoStruct* aForwardDistancesInfo);
void drawCollisionDecision(int aDegreeToTurn, uint8_t aLengthOfVector, bool aDoClear);

void ServoWrite(uint8_t aValue,bool doDelay);

/*
 * Values for pro implementation
 */
const int CENTIMETER_PER_RIDE_PRO = 25;

// do not measure and process distances greater than 100 cm
#define US_TIMEOUT_CENTIMETER_PRO 100

// I measured ca. 110 ms
const int MILLIS_FOR_SERVO_20_DEGREES = 120;

void clearPrintedForwardDistancesInfos();
bool fillForwardDistancesInfo(ForwardDistancesInfoStruct* aForwardDistancesInfo, bool aShowValues,
        bool aDoFirstValue);
void doWallDetection(ForwardDistancesInfoStruct* aForwardDistancesInfo, bool aShowValues);
int doCollisionDetectionPro(ForwardDistancesInfoStruct* aForwardDistancesInfo);
void driveAutonomous(bool (*afillForwardDistancesInfoFunction)(ForwardDistancesInfoStruct*, bool, bool),
        int (*aCollisionDetectionFunction)(ForwardDistancesInfoStruct*));

#endif /* SRC_AUTONOMOUSDRIVE_H_ */

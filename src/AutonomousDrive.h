/*
 * AutonomousDrive.h
 *
 *  Created on: 08.11.2016
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 */

#ifndef SRC_AUTONOMOUSDRIVE_H_
#define SRC_AUTONOMOUSDRIVE_H_

#include "Servo.h"
#include <stdint.h>

#define DEGREE_PER_STEP 20
#define NUMBER_OF_DISTANCES ((180 / DEGREE_PER_STEP) + 1)
#define MAX_DISTANCES_INDEX (NUMBER_OF_DISTANCES - 1)

#define INDEX_FORWARD_1 4
#define INDEX_FORWARD_2 5
#define INDEX_RIGHT 0
#define INDEX_LEFT MAX_DISTANCES_INDEX

struct ForwardDistancesInfoStruct {
    uint8_t RawDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degree
    uint8_t ProcessedDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degree
    uint8_t IndexOfMaxDistance;
    uint8_t IndexOfMinDistance;
    uint8_t MaxDistance;
    uint8_t MinDistance;
};

extern ForwardDistancesInfoStruct ForwardDistancesInfo;

extern int sDegreeToTurn;
extern uint8_t sLastServoAngleInDegree; // needed for optimized delay for servo repositioning

extern uint16_t sCountPerScan;

void drawForwardDistancesInfos(ForwardDistancesInfoStruct* aForwardDistancesInfo);
void ServoWrite(Servo aServo, int aValue);

/*
 * Values for pro implementation
 */
const int CENTIMETER_PER_RIDE_PRO = 20;
const int MIN_ALLOWED_DISTANCE_TO_FRONT_PRO = 50;
const int MIN_ALLOWED_DISTANCE_TO_SIDE_PRO = 25;
const int DISTANCE_FOR_WALL_DETECT = 60;

// do not measure and process distances greater than 100 cm
#define US_TIMEOUT_CENTIMETER_PRO 100

// I measured ca. 110 ms
const int MILLIS_FOR_SERVO_30_DEGREES = 120;

void clearPrintedForwardDistancesInfos();
bool fillForwardDistancesInfoPro(ForwardDistancesInfoStruct* aForwardDistancesInfo, Servo aServoForUS, bool aShowValues,
        bool aDoFirstValue);
void doWallDetectionPro(ForwardDistancesInfoStruct* aForwardDistancesInfo);
int doCollisionDetectionPro(ForwardDistancesInfoStruct* aForwardDistancesInfo);
void driveAutonomous(bool * aRunFlagPtr, bool (*afillForwardDistancesInfoFunction)(ForwardDistancesInfoStruct*, Servo, bool, bool),
        int (*aCollisionDetectionFunction)(ForwardDistancesInfoStruct*));

#endif /* SRC_AUTONOMOUSDRIVE_H_ */

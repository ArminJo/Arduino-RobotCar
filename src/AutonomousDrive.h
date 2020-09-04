/*
 * AutonomousDrive.h
 *
 *  Created on: 08.11.2016
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

#ifndef SRC_AUTONOMOUSDRIVE_H_
#define SRC_AUTONOMOUSDRIVE_H_

#include <Servo.h>
#include <stdint.h>

/*
 * Different autonomous driving modes
 */
#define MODE_MANUAL_DRIVE 0
#define MODE_AUTONOMOUS_DRIVE_BUILTIN 1
#define MODE_AUTONOMOUS_DRIVE_USER 2
#define MODE_FOLLOWER 3
extern uint8_t sDriveMode;

/*
 * Step modes for MODE_AUTONOMOUS_DRIVE
 */
#define MODE_CONTINUOUS 0
#define MODE_STEP_TO_NEXT_TURN 1 // stop before a turn
#define MODE_SINGLE_STEP 2 // stop after CENTIMETER_PER_RIDE_2
extern uint8_t sStepMode;
extern bool sDoStep;

#define FOLLOWER_MIN_DISTANCE  22
#define FOLLOWER_MAX_DISTANCE  30
#define FOLLOWER_RESCAN_DISTANCE  45 // search if target moved to side

/*
 * Constants for fillAndShowForwardDistancesInfo(), doWallDetection etc.
 */
#define DEGREES_PER_STEP  18
#define STEPS_PER_SCAN    9 // -> 162 degrees for 18 DEGREES_PER_STEP
#define NUMBER_OF_DISTANCES (STEPS_PER_SCAN + 1)
#define START_DEGREES      ((180 - (DEGREES_PER_STEP * STEPS_PER_SCAN)) / 2) // 9 - we need it symmetrical in the 180 degrees range
extern bool sDoSlowScan;
extern uint8_t sLastServoAngleInDegrees; // needed for optimized delay for servo repositioning
extern Servo DistanceServo;

/*
 * Different result types acquired at one scan
 */
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
#define SCAN_MODE_MINIMUM  0
#define SCAN_MODE_MAXIMUM  1
#define SCAN_MODE_US    2
#define SCAN_MODE_IR    3
extern uint8_t sScanMode;
#endif

#define GO_BACK_AND_SCAN_AGAIN 360 // possible result of doBuiltInCollisionDetection()

/*
 * Index definitions for ForwardDistancesInfoStruct
 */
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

extern uint8_t sCentimeterPerScanTimesTwo; // Statistics
extern uint8_t sCentimeterPerScan; // = sCentimeterPerScanTimesTwo / 2

void doAutonomousDrive();
void driveAutonomousOneStep();
void driveFollowerModeOneStep();

void initDistanceServo();
void DistanceServoWriteAndDelay(uint8_t aValue, bool doDelay = false);

unsigned int getDistanceAsCentiMeter();

bool fillAndShowForwardDistancesInfo(bool aShowValues, bool aDoFirstValue, bool aForceScan = false);
void doWallDetection(bool aShowValues);
int doBuiltInCollisionDetection();

#endif /* SRC_AUTONOMOUSDRIVE_H_ */

#pragma once

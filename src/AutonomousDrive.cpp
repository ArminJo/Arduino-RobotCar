/*
 * AutonomousDrive.cpp
 *
 * Contains:
 * fillForwardDistancesInfoPro(): Acquisition of 180 degrees distances by ultrasonic sensor and servo
 * doWallDetection(): Enhancement of acquired data because of lack of detecting flat surfaces by US at angels out of 70 to 110 degree.
 * doBuiltInCollisionDetection(): decision where to turn in dependency of the acquired distances.
 * driveAutonomousOneStep(): The loop which handles the start/stop, single step and path output functionality.
 *
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include "AutonomousDrive.h"

#include "RobotCar.h"
#include "RobotCarGui.h"

#include "HCSR04.h"
#include "Distance.h"

ForwardDistancesInfoStruct sForwardDistancesInfo;

Servo DistanceServo;
uint8_t sLastServoAngleInDegrees; // 0 - 180 needed for optimized delay for servo repositioning

// Storage for turning decision especially for single step mode
int sNextDegreesToTurn = 0;
// Storage of last turning for insertToPath()
int sLastDegreesTurned = 0;

uint8_t sTurnMode = TURN_IN_PLACE;

uint8_t sCentimeterPerScanTimesTwo = CENTIMETER_PER_RIDE * 2; // = encoder counts per scan
uint8_t sCentimeterPerScan = CENTIMETER_PER_RIDE;

void initUSServo() {
    DistanceServo.attach(DISTANCE_SERVO_PIN);
    DistanceServoWriteAndDelay(90);
}

//#define USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
/*
 * sets also sLastServoAngleInDegrees to enable optimized servo movement and delays
 * SG90 Micro Servo has reached its end position if the current (200 mA) is low for more than 11 to 14 ms
 */
void DistanceServoWriteAndDelay(uint8_t aTargetDegrees, bool doDelay) {

    if (aTargetDegrees > 220) {
        // handle underflow
        aTargetDegrees = 0;
    } else if (aTargetDegrees > 180) {
        // handle underflow
        aTargetDegrees = 180;
    }

    uint8_t tDeltaDegrees;
#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
    int8_t tOvershootDegrees; // Experimental
#endif
    uint8_t tLastServoAngleInDegrees = sLastServoAngleInDegrees;
    sLastServoAngleInDegrees = aTargetDegrees;

    if (tLastServoAngleInDegrees == aTargetDegrees) {
        return;
    } else if (aTargetDegrees > tLastServoAngleInDegrees) {
        tDeltaDegrees = aTargetDegrees - tLastServoAngleInDegrees;
#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
        tOvershootDegrees = 3; // Experimental
#endif
    } else {
        tDeltaDegrees = tLastServoAngleInDegrees - aTargetDegrees;
#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
        tOvershootDegrees = -3; // Experimental
#endif
    }

#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
    /*
     * Experimental!
     * Compensate (set target to more degrees) for fast servo speed
     * Reasonable value is between 2 and 3 at 20 degrees and tWaitDelayforServo = tDeltaDegrees * 5
     * Reasonable value is between 10 and 20 degrees and tWaitDelayforServo = tDeltaDegrees * 4 => avoid it
     */
    if (!sDoSlowScan) {
        aTargetDegrees += tOvershootDegrees;
    }
#endif

    // My servo is top down and therefore inverted
    aTargetDegrees = 180 - aTargetDegrees;
    DistanceServo.write(aTargetDegrees);
    if (doDelay) {
// Synchronize before doing delay
        rightEncoderMotor.synchronizeMotor(&leftEncoderMotor, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
// Datasheet says: SG90 Micro Servo needs 100 millis per 60 degrees angle => 300 ms per 180
// I measured: SG90 Micro Servo needs 400 per 180 degrees and 400 per 2*90 degree, but 540 millis per 9*20 degree
// 60-80 ms for 20 degrees

//        // wait at least 5 ms for the servo to receive signal
//        delay(SERVO_INITIAL_DELAY);
//        digitalWrite(DEBUG_OUT_PIN, LOW);

        /*
         * Factor 8 gives a fairly reproducible US result, but some dropouts for IR
         * factor 7 gives some strange (to small) values for US.
         *
         */
// , factor 4 is a bit too fast, even 7 gives some strange values
        uint16_t tWaitDelayforServo;
        if (sDoSlowScan) {
            tWaitDelayforServo = tDeltaDegrees * 16; // 16 => 288 ms for 18 degrees
        } else {
#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
            tWaitDelayforServo = tDeltaDegrees * 5;
#else
#  ifdef CAR_HAS_IR_DISTANCE_SENSOR
            tWaitDelayforServo = tDeltaDegrees * 9; // 9 => 162 ms for 18 degrees
#  else
            tWaitDelayforServo = tDeltaDegrees * 8; // 7 => 128 ms, 8 => 144 for 18 degrees
#  endif
#endif
        }
        delayAndLoopGUI(tWaitDelayforServo);
    }
}

/*
 * Get 7 distances starting at 9 degrees (right) increasing by 18 degrees up to 171 degrees (left)
 * Avoid 0 and 180 degrees since at this position the US sensor might see the wheels of the car as an obstacle.
 * @param aDoFirstValue if false, skip first value since it is the same as last value of last measurement in continuous mode.
 *
 * Wall detection:
 * If 2 or 3 adjacent values are quite short and the surrounding values are quite far,
 * then assume a wall which cannot reflect the pulse for the surrounding values.
 *
 */
void __attribute__((weak)) fillAndShowForwardDistancesInfo(bool aShowValues, bool aDoFirstValue) {

    color16_t tColor;

// Values for forward scanning
    uint8_t tCurrentDegrees = START_DEGREES;
    int8_t tDegreeIncrement = DEGREES_PER_STEP;
    int8_t tIndex = 0;
    int8_t tIndexDelta = 1;
    if (sLastServoAngleInDegrees >= 170) {
// values for backward scanning
        tCurrentDegrees = 180 - START_DEGREES;
        tDegreeIncrement = -(tDegreeIncrement);
        tIndex = STEPS_PER_SCAN;
        tIndexDelta = -1;
    }
    if (!aDoFirstValue) {
// skip first value, since it is equal to last value of last measurement
        tIndex += tIndexDelta;
        tCurrentDegrees += tDegreeIncrement;
    }

    while (tIndex >= 0 && tIndex < NUMBER_OF_DISTANCES) {
        /*
         * rotate servo, wait and get distance
         */
        DistanceServoWriteAndDelay(tCurrentDegrees, true);
#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
        if (sScanMode == SCAN_MODE_BOTH || sScanMode == SCAN_MODE_IR) {
            sToFDistanceSensor.startRanging();
        }
#endif

        unsigned int tCentimeter = getUSDistanceAsCentiMeterWithCentimeterTimeout(DISTANCE_TIMEOUT_CM);
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
        unsigned int tIRCentimeter;
        if (sScanMode == SCAN_MODE_BOTH || sScanMode == SCAN_MODE_IR) {
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR)
        tIRCentimeter = getIRDistanceAsCentimeter();
#  elif defined(CAR_HAS_TOF_DISTANCE_SENSOR)
            tIRCentimeter = readToFDistanceAsCentimeter();
#  endif
            if (sScanMode == SCAN_MODE_IR) {
                tCentimeter = tIRCentimeter;
            } else {
                // Scan mode BOTH => Take the minimum of the two values
                if (tCentimeter > tIRCentimeter) {
                    tCentimeter = tIRCentimeter;
                }
            }
        }
#endif
        if (((tIndex == INDEX_FORWARD_1 || tIndex == INDEX_FORWARD_2) && tCentimeter <= sCentimeterPerScanTimesTwo)
                || (!sRuningAutonomousDrive)) {
            /*
             * Emergency stop
             */
            RobotCar.stopCar();
        }

        if (aShowValues) {
            /*
             * Determine color
             */
            tColor = COLOR_RED; // tCentimeter <= sCentimeterPerScan
            if (tCentimeter >= DISTANCE_TIMEOUT_CM) {
                tColor = DISTANCE_TIMEOUT_COLOR;
            } else if (tCentimeter > sCentimeterPerScanTimesTwo) {
                tColor = COLOR_GREEN;
            } else if (tCentimeter > sCentimeterPerScan) {
                tColor = COLOR_YELLOW;
            }

            /*
             * Clear old and draw new line
             */
            BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y,
                    sForwardDistancesInfo.RawDistancesArray[tIndex], tCurrentDegrees, COLOR_WHITE, 3);
            BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tCentimeter, tCurrentDegrees, tColor,
                    3);
        }
        /*
         * Store value and search for min and max
         */
        sForwardDistancesInfo.RawDistancesArray[tIndex] = tCentimeter;

        tIndex += tIndexDelta;
        tCurrentDegrees += tDegreeIncrement;
    }
}

/*
 * Find min and max value. Prefer the headmost value if we have more than one choice
 */
void doPostProcess() {
    unsigned int tMax = 0;
    unsigned int tMin = __UINT16_MAX__; // = 65535
    for (uint8_t i = 0; i < (NUMBER_OF_DISTANCES + 1) / 2; ++i) {
        uint8_t tDistance = sForwardDistancesInfo.ProcessedDistancesArray[i];
        uint8_t tIndex = i;
        for (int j = 0; j < 2; ++j) {
            if (tDistance >= tMax) {
                tMax = tDistance;
                sForwardDistancesInfo.IndexOfMaxDistance = tIndex;
                sForwardDistancesInfo.MaxDistance = tDistance;
            }
            if (tDistance <= tMin) {
                tMin = tDistance;
                sForwardDistancesInfo.IndexOfMinDistance = tIndex;
                sForwardDistancesInfo.MinDistance = tDistance;
            }
            tIndex = STEPS_PER_SCAN - i;
            tDistance = sForwardDistancesInfo.ProcessedDistancesArray[tIndex];
        }
    }
}

/*
 * This documentation assumes 20 degrees stepping.
 * By changing DEGREES_PER_STEP it can easily adopted to other degrees values.
 *
 * Assume the value of 20 and 40 degrees are distances to a wall.
 * @return the clipped (aClipValue) distance to the wall of the vector at 0 degree.
 *
 * @param [out] aDegreeOfEndpointConnectingLine: The angle of the line from endpoint 0 degrees to given endpoints
 * 0 means x values of given endpoints are the same >= wall is parallel / rectangular to the vector at 0 degree
 * Positive means wall is more ore less in front, to avoid we must turn positive angle
 * 90 means y values are the same =>  wall is in front
 * Negative means we are heading away from wall
 */
uint8_t computeNeigbourValue(uint8_t aDegreesPerStepValue, uint8_t a2DegreesPerStepValue, uint8_t aClipValue,
        int8_t * aDegreeOfEndpointConnectingLine) {

    /*
     * Name of the variables are for DEGREES_PER_STEP = 20 only for better understanding.
     * The computation of course works for other values of DEGREES_PER_STEP!
     */
    float tYat20degrees = sin((PI / 180) * DEGREES_PER_STEP) * aDegreesPerStepValue; // e.g. 20 Degree
// assume current = 40 Degree
    float tYat40degrees = sin((PI / 180) * (DEGREES_PER_STEP * 2)) * a2DegreesPerStepValue; // e.g. 40 Degree

//    char tStringBuffer[] = "A=_______ L=_______";
//    dtostrf(tYat40degrees, 7, 2, &tStringBuffer[2]);
//    tStringBuffer[9] = ' ';
//    dtostrf(tYat20degrees, 7, 2, &tStringBuffer[12]);
//    BlueDisplay1.debugMessage(tStringBuffer);

    uint8_t tDistanceAtZeroDegree = aClipValue;

    /*
     * if tY40degrees == tY20degrees the tInvGradient is infinite (distance at 0 is infinite)
     */
    if (tYat40degrees > tYat20degrees) {
        float tXat20degrees = cos((PI / 180) * DEGREES_PER_STEP) * aDegreesPerStepValue; // 20 Degree
        float tXat40degrees = cos((PI / 180) * (DEGREES_PER_STEP * 2)) * a2DegreesPerStepValue; // 40 Degree

//        dtostrf(tXat40degrees, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tXat20degrees, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

        /*  Here the graphic for 90 and 60 degrees (since we have no ASCII graphic symbols for 20 and 40 degree)
         *  In this function we have e.g. 40 and 20 degrees and compute 0 degrees!
         *      90 degrees value
         *      |\ \==wall
         *      | \  60 degrees value
         *      | /\
 *      |/__\ 0 degrees value to be computed
         */

        /*
         * InvGradient line represents the wall
         * if tXat20degrees > tXat40degrees InvGradient is negative => X0 value is bigger than X20 one / right wall is in front if we look in 90 degrees direction
         * if tXat20degrees == tXat40degrees InvGradient is 0 / wall is parallel right / 0 degree
         * if tXat20degrees < tXat40degrees InvGradient is positive / right wall is behind / degrees is negative (from direction front which is 90 degrees)
         */
        float tInvGradient = (tXat40degrees - tXat20degrees) / (tYat40degrees - tYat20degrees);
        float tXatZeroDegree = tXat20degrees - (tInvGradient * tYat20degrees);
        *aDegreeOfEndpointConnectingLine = -(atan(tInvGradient) * RAD_TO_DEG);
//        tStringBuffer[0] = 'G';
//        tStringBuffer[10] = 'B';
//        dtostrf(tInvGradient, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tXZeroDegree, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

        if (tXatZeroDegree < 255) {
            tDistanceAtZeroDegree = tXatZeroDegree + 0.5;
            if (tDistanceAtZeroDegree > aClipValue) {
                tDistanceAtZeroDegree = aClipValue;
            }
        }
    }
    return tDistanceAtZeroDegree;
}

/*
 * The problem of the ultrasonic values is, that you can only detect a wall with the ultrasonic sensor if the angle of the wall relative to sensor axis is approximately between 70 and 110 degree.
 * For other angels the reflected ultrasonic beam can not not reach the receiver which leads to unrealistic great distances.
 *
 * Therefore I take samples every 18 degrees and if I get 2 adjacent short (< sCentimeterPerScanTimesTwo) distances, I assume a wall determined by these 2 samples.
 * The (invalid) values 18 degrees right and left of these samples are then extrapolated by computeNeigbourValue().
 */
//#define TRACE // only used for this function
void doWallDetection(bool aShowValues) {
    uint8_t tTempDistancesArray[NUMBER_OF_DISTANCES];
    /*
     * First copy all raw values
     */
    memcpy(tTempDistancesArray, sForwardDistancesInfo.RawDistancesArray, NUMBER_OF_DISTANCES);

    uint8_t tCurrentAngleToCheck = START_DEGREES + (2 * DEGREES_PER_STEP); // first angle to adjust at index 2
    int8_t tDegreeOfConnectingLine;
    sForwardDistancesInfo.WallRightAngleDegrees = 0;
    sForwardDistancesInfo.WallLeftAngleDegrees = 0;

    /*
     * Parse the array from 0 to STEPS_PER_SCAN
     * Check values at i and i-1 and adjust value at i+1
     * i is index of CurrentValue
     */
    uint8_t tLastDistance = tTempDistancesArray[0];
    uint8_t tCurrentDistance = tTempDistancesArray[1];
    for (uint8_t i = 1; i < STEPS_PER_SCAN; ++i) {
        uint8_t tNextDistanceOriginal = tTempDistancesArray[i + 1];
        if (tLastDistance < sCentimeterPerScanTimesTwo && tCurrentDistance < sCentimeterPerScanTimesTwo) {
            /*
             * 2 adjacent short distances -> assume a wall -> adjust adjacent values
             */

            /*
             * Use computeNeigbourValue the other way round
             * i.e. put 20 degrees to 40 degrees parameter and vice versa in order to use the 0 degree value as the 60 degrees one
             */
            uint8_t tNextDistanceComputed = computeNeigbourValue(tCurrentDistance, tLastDistance, DISTANCE_TIMEOUT_CM,
                    &tDegreeOfConnectingLine);
#ifdef TRACE
            BlueDisplay1.debug("i=", i);
            BlueDisplay1.debug("AngleToCheck @i+1=", tCurrentAngleToCheck);
            BlueDisplay1.debug("Original distance @i+1=", tNextDistanceOriginal);
            BlueDisplay1.debug("New distance @i+1=", tNextDistanceComputed);
            BlueDisplay1.debug("Connecting degrees @i+1=", tDegreeOfConnectingLine);
#endif

            if (tNextDistanceOriginal > tNextDistanceComputed + 5) {
                /*
                 * Adjust and draw next value if computed value is less than original value - 5
                 *
                 * Start with i=1 and adjust for 2 at (2 * DEGREES_PER_STEP) + START_DEGREES.
                 * Since we use computeNeigbourValue the other way round, we must change sign of tDegreeOfConnectingLine!
                 * The formula is 90 - (180->sum of degrees in triangle - tCurrentAngleToCheck - (90 - tDegreeOfConnectingLine)->since we use it the other way round)
                 * Which leads to -90 + tCurrentAngleToCheck + 90 - tDegreeOfConnectingLine.
                 * If we then get a tDegreeOfConnectingLine value of 0 we have a wall at right rectangular to the vector at 2,
                 * Negative raw values means the wall is more in front / the the wall angle is greater,
                 */
                int tDegreeOfWallAngle = tCurrentAngleToCheck - tDegreeOfConnectingLine;
#ifdef TRACE
                BlueDisplay1.debug("tDegreeOfWallAngle=", tDegreeOfWallAngle);
#endif
                if (tDegreeOfWallAngle <= 90) {
                    // wall at right
                    sForwardDistancesInfo.WallRightAngleDegrees = tDegreeOfWallAngle;
                } else {
                    // wall at left
                    sForwardDistancesInfo.WallLeftAngleDegrees = 180 - tDegreeOfWallAngle;
                }

                // store and draw adjusted value
                tTempDistancesArray[i + 1] = tNextDistanceComputed;
                tNextDistanceOriginal = tNextDistanceComputed;
                if (aShowValues) {
                    BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextDistanceComputed,
                            tCurrentAngleToCheck, COLOR_WHITE, 1);
                }
            }
        }
        tLastDistance = tCurrentDistance;
        tCurrentDistance = tNextDistanceOriginal;
        tCurrentAngleToCheck += DEGREES_PER_STEP;
    }

    /*
     * Go backwards through the array
     */
    memcpy(sForwardDistancesInfo.ProcessedDistancesArray, tTempDistancesArray, NUMBER_OF_DISTANCES);

    tLastDistance = tTempDistancesArray[STEPS_PER_SCAN];
    tCurrentDistance = tTempDistancesArray[STEPS_PER_SCAN - 1];
    tCurrentAngleToCheck = 180 - (START_DEGREES + (2 * DEGREES_PER_STEP));

    /*
     * check values at i and i+1 and adjust value at i-1
     */
    for (uint8_t i = STEPS_PER_SCAN - 1; i > 0; --i) {
        uint8_t tNextValue = tTempDistancesArray[i - 1];

// Do it only if none of the 3 values are processed before
        if (tTempDistancesArray[i + 1] == sForwardDistancesInfo.RawDistancesArray[i + 1]
                && tTempDistancesArray[i] == sForwardDistancesInfo.RawDistancesArray[i]
                && tNextValue == sForwardDistancesInfo.RawDistancesArray[i - 1]) {

            /*
             * check values at i+1 and i and adjust value at i-1
             */
            if (tLastDistance < sCentimeterPerScanTimesTwo && tCurrentDistance < sCentimeterPerScanTimesTwo) {
                /*
                 * Wall detected -> adjust adjacent values
                 * Use computeNeigbourValue in the intended way, so do not change sign of tDegreeOfConnectingLine!
                 */
                uint8_t tNextValueComputed = computeNeigbourValue(tCurrentDistance, tLastDistance, DISTANCE_TIMEOUT_CM,
                        &tDegreeOfConnectingLine);
#ifdef TRACE
                BlueDisplay1.debug("i=", i);
                BlueDisplay1.debug("AngleToCheck @i+1=", tCurrentAngleToCheck);
                BlueDisplay1.debug("Original distance @i-1=", tNextValue);
                BlueDisplay1.debug("New distance @i-1=", tNextValueComputed);
                BlueDisplay1.debug("Connecting degrees @i-1=", tDegreeOfConnectingLine);
#endif
                if (tNextValue > tNextValueComputed + 5) {
                    // start with i = 8 and adjust for 7
                    // degrees at index i-1 are ((i - 1) * DEGREES_PER_STEP) + START_DEGREES
                    int tWallBackwardDegrees = tCurrentAngleToCheck + tDegreeOfConnectingLine;
#ifdef TRACE
                    BlueDisplay1.debug("tWallBackwardDegrees=", tWallBackwardDegrees);
#endif
                    if (tWallBackwardDegrees <= 90) {
                        // wall at right - overwrite only if greater
                        if (sForwardDistancesInfo.WallRightAngleDegrees < tWallBackwardDegrees) {
                            sForwardDistancesInfo.WallRightAngleDegrees = tWallBackwardDegrees;
#ifdef TRACE
                        BlueDisplay1.debug("WallRightAngleDegrees=", sForwardDistancesInfo.WallRightAngleDegrees);
#endif
                        }
                    } else if (sForwardDistancesInfo.WallLeftAngleDegrees < (180 - tWallBackwardDegrees)) {
                        // wall at right - overwrite only if greater
                        sForwardDistancesInfo.WallLeftAngleDegrees = 180 - tWallBackwardDegrees;
#ifdef TRACE
                        BlueDisplay1.debug("WallLeftAngleDegrees=", sForwardDistancesInfo.WallLeftAngleDegrees);
#endif

                    }
                    //Adjust and draw next value if original value is greater
                    sForwardDistancesInfo.ProcessedDistancesArray[i - 1] = tNextValueComputed;
                    tNextValue = tNextValueComputed;
                    if (aShowValues) {
                        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextValueComputed,
                                tCurrentAngleToCheck, COLOR_WHITE, 1);
                    }
                }
            }
        }
        tLastDistance = tCurrentDistance;
        tCurrentDistance = tNextValue;
        tCurrentAngleToCheck -= DEGREES_PER_STEP;

    }
    doPostProcess();
}

#define GO_BACK_AND_SCAN_AGAIN 360
/*
 * Checks distances and returns degrees to turn
 * 0 -> no turn, > 0 -> turn left, < 0 -> turn right, > 360 go back, since too close to wall
 */
int doBuiltInCollisionDetection() {
    int tDegreeToTurn = 0;
// 5 is too low
    if (sForwardDistancesInfo.MinDistance < 7) {
        /*
         * Min Distance too small => go back and scan again
         */
        return GO_BACK_AND_SCAN_AGAIN;
    }
    /*
     * First check if free ahead
     */
    if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_1] > sCentimeterPerScanTimesTwo
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_2] > sCentimeterPerScanTimesTwo) {
        /*
         * Free ahead, check if our side is near to the wall and make corrections
         */
        if (sForwardDistancesInfo.WallRightAngleDegrees != 0 || sForwardDistancesInfo.WallLeftAngleDegrees != 0) {
            /*
             * Wall detected
             */
            if (sForwardDistancesInfo.WallRightAngleDegrees > sForwardDistancesInfo.WallLeftAngleDegrees) {
                /*
                 * Wall at right => turn left
                 */
                tDegreeToTurn = sForwardDistancesInfo.WallRightAngleDegrees;
            } else {
                /*
                 * Wall at left => turn right
                 */
                tDegreeToTurn = -sForwardDistancesInfo.WallLeftAngleDegrees;
            }

        }
    } else {
        if (sForwardDistancesInfo.WallRightAngleDegrees != 0 || sForwardDistancesInfo.WallLeftAngleDegrees != 0) {
            /*
             * Wall detected
             */
            if (sForwardDistancesInfo.WallRightAngleDegrees > sForwardDistancesInfo.WallLeftAngleDegrees) {
                /*
                 * Wall at right => turn left
                 */
                tDegreeToTurn = sForwardDistancesInfo.WallRightAngleDegrees;
            } else {
                /*
                 * Wall at left => turn right
                 */
                tDegreeToTurn = -sForwardDistancesInfo.WallLeftAngleDegrees;
            }
        } else {
            /*
             * Not free ahead, must turn, check if another forward direction is suitable
             */
            if (sForwardDistancesInfo.MaxDistance > sCentimeterPerScanTimesTwo) {
                /*
                 * Go to max distance
                 */
                tDegreeToTurn = sForwardDistancesInfo.IndexOfMaxDistance * DEGREES_PER_STEP - 90;
            } else {
                /*
                 * Max distances are all too short => must go back / turn by 180 degree
                 */
                tDegreeToTurn = 180;
            }
        }
    }
    return tDegreeToTurn;
}

/*
 * Do one step of autonomous driving
 * Compute sNextDegreesToTurn AFTER the movement to be able to stop before next turn
 * 1. Check for step conditions if step should happen
 *
 */
void driveAutonomousOneStep(int (*aCollisionDetectionFunction)()) {

    /*
     * 1. Check for step conditions if step should happen
     */
    if (sStepMode == MODE_CONTINUOUS || (sStepMode == MODE_SINGLE_STEP && sDoStep)
            || (sStepMode == MODE_STEP_TO_NEXT_TURN && (!RobotCar.isStopped() || sDoStep))) {
        /*
         * Do one step
         */
        bool tMovementJustStarted = sDoStep; // tMovementJustStarted is needed for speeding up US scanning by skipping first scan angle if not just started.
        sDoStep = false; // Now it can be set again by GUI

        /*
         * Turn and start car if needed
         */
        int tLastDisplayedDegreeToTurn = sNextDegreesToTurn;
        if (sStepMode == MODE_SINGLE_STEP) {
            /*
             * SINGLE_STEP -> optional turn and go fixed distance
             * Do not turn after movement to enable analysis of turn decision
             */
            if (sNextDegreesToTurn == GO_BACK_AND_SCAN_AGAIN) {
                RobotCar.goDistanceCentimeter(-10, &loopGUI);
            } else {
                RobotCar.rotateCar(sNextDegreesToTurn, sTurnMode);
                // wait to really stop after turning
                delay(100);
                sLastDegreesTurned = sNextDegreesToTurn;
                sNextDegreesToTurn = 0;
                // and go
                RobotCar.goDistanceCentimeter(CENTIMETER_PER_RIDE, &loopGUI);
            }
        } else
        /*
         * Step of MODE_STEP_TO_NEXT_TURN or start of MODE_CONTINUOUS
         * Rotate / go backwards and start
         */
        if (RobotCar.isStopped()) {
            if (sNextDegreesToTurn == GO_BACK_AND_SCAN_AGAIN) {
                // go backwards
                RobotCar.goDistanceCentimeter(-10, &loopGUI);
            } else {
                // rotate
                RobotCar.rotateCar(sNextDegreesToTurn, sTurnMode);
                // wait to really stop after turning
                delay(100);
                sLastDegreesTurned = sNextDegreesToTurn;
                sNextDegreesToTurn = 0;
                // and go
                RobotCar.startAndWaitForFullSpeed();
                tMovementJustStarted = true;
            }
        }

        /*
         * Here car is moving
         */

        uint16_t tStepStartDistanceCount = rightEncoderMotor.DistanceCount;

        bool tCurrentPageIsAutomaticControl = (sCurrentPage == PAGE_AUTOMATIC_CONTROL);

//Clear old decision marker by redrawing it with a white line
        if (tCurrentPageIsAutomaticControl) {
            drawCollisionDecision(tLastDisplayedDegreeToTurn, sCentimeterPerScan, true);
        }

        /*
         * The magic happens HERE
         * This runs as fast as possible and mainly determine the duration of one step
         */
        fillAndShowForwardDistancesInfo(tCurrentPageIsAutomaticControl, tMovementJustStarted);
        doWallDetection(tCurrentPageIsAutomaticControl);
        sNextDegreesToTurn = aCollisionDetectionFunction();
        drawCollisionDecision(sNextDegreesToTurn, sCentimeterPerScan, false);

        /*
         * compute distance driven for one US scan
         */
        if (!RobotCar.isStopped()) {
            /*
             * No stop here => distance is valid
             */
            sCentimeterPerScanTimesTwo = rightEncoderMotor.DistanceCount - tStepStartDistanceCount;
            sCentimeterPerScan = sCentimeterPerScanTimesTwo / 2;
            if (tCurrentPageIsAutomaticControl) {
                char tStringBuffer[6];
                sprintf_P(tStringBuffer, PSTR("%2d%s"), sCentimeterPerScan, "cm");
                BlueDisplay1.drawText(0, BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND, tStringBuffer, TEXT_SIZE_11,
                COLOR_BLACK, COLOR_WHITE);
            }
        }

        /*
         * Handle stop of car and path data
         */
        if ((sNextDegreesToTurn != 0 && sStepMode == MODE_STEP_TO_NEXT_TURN) || sStepMode == MODE_SINGLE_STEP) {
            /*
             * Stop if rotation requested or single step => insert / update last ride in path
             */
            RobotCar.stopCar();
            if (sStepMode == MODE_SINGLE_STEP) {
                insertToPath(CENTIMETER_PER_RIDE * 2, sLastDegreesTurned, true);
            } else {
                // add last driven distance to path
                insertToPath(rightEncoderMotor.LastRideDistanceCount, sLastDegreesTurned, true);
            }
        } else {
            /*
             * No stop, just continue => overwrite last path element with current riding distance and try to synchronize motors
             */
            insertToPath(rightEncoderMotor.DistanceCount, sLastDegreesTurned, false);
            rightEncoderMotor.synchronizeMotor(&leftEncoderMotor, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
        }

        if (sCurrentPage == PAGE_SHOW_PATH) {
            drawPathInfoPage();
        }
    }
}


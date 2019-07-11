/*
 * AutonomousDrive.cpp
 *
 * Contains:
 * fillForwardDistancesInfoPro(): Acquisition of 180 degrees distances by ultrasonic sensor and servo
 * doWallDetection(): Enhancement of acquired data because of lack of detecting flat surfaces by US at angels out of 70 to 110 degree.
 * doBuiltInCollisionDetection(): decision where to turn in dependency of the acquired distances.
 * driveAutonomousOneStep(): The loop which handles the start/stop, single step and path output functionality.
 *
 *  Created on: 08.11.2016
 *  Copyright (C) 2016  Armin Joachimsmeyer
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

#include <EncoderMotor.h>
#include <HCSR04.h>

#include "AutonomousDrive.h"
#include "RobotCar.h"
#include "RobotCarGui.h"

#include <stdlib.h> // for dtostrf()

ForwardDistancesInfoStruct sForwardDistancesInfo;

Servo USDistanceServo;
uint8_t sLastServoAngleInDegrees; // 0 - 180 needed for optimized delay for servo repositioning

// Storage for turning decision especially for single step mode
int sNextDegreesToTurn = 0;
// Storage of last turning for insertToPath()
int sLastDegreesTurned = 0;

// TODO handle turn modes
uint8_t sTurnMode = TURN_IN_PLACE;

uint8_t sCountPerScan = CENTIMETER_PER_RIDE * 2;
uint8_t sCentimeterPerScan = CENTIMETER_PER_RIDE;

void initUSServo() {
    USDistanceServo.attach(US_SERVO_PIN);
    US_ServoWriteAndDelay(90);
}
/*
 * sets also sLastServoAngleInDegrees to enable optimized servo movement and delays
 * SG90 Micro Servo has reached its end position if the current (200 mA) is low for more than 11 to 14 ms
 */
void US_ServoWriteAndDelay(uint8_t aValueDegrees, bool doDelay) {

    if (aValueDegrees > 220) {
        // handle underflow
        aValueDegrees = 0;
    } else if (aValueDegrees > 180) {
        // handle underflow
        aValueDegrees = 180;
    }
    uint8_t tDeltaDegrees = abs(sLastServoAngleInDegrees - aValueDegrees);
    sLastServoAngleInDegrees = aValueDegrees;
    // My servo is top down and therefore inverted
    aValueDegrees = 180 - aValueDegrees;
    USDistanceServo.write(aValueDegrees);
    if (tDeltaDegrees == 0) {
        return;
    }
    if (doDelay) {
        // Synchronize and check for user input before doing delay
        rightEncoderMotor.synchronizeMotor(&leftEncoderMotor, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
        loopGUI();
        // Datasheet says: SG90 Micro Servo needs 100 millis per 60 degrees angle => 300 ms per 180
        // I measured: SG90 Micro Servo needs 400 per 180 degrees and 400 per 2*90 degree, but 540 millis per 9*20 degree
        // 60-80 ms for 20 degrees

//        // wait at least 5 ms for the servo to receive signal
//        delay(SERVO_INITIAL_DELAY);
//        digitalWrite(DEBUG_OUT_PIN, LOW);

        // factor 8 gives a fairly reproducible result, factor 4 is a bit too fast
        uint16_t tWaitDelayforServo = tDeltaDegrees * 7;
        delay(tWaitDelayforServo);
    }
}

/*
 * Get 7 distances starting at 10 degree (right) increasing by 18 degrees up to 170 degrees (left)
 * Avoid 0 and 180 degree since at this position the US sensor might see the wheels of the car as an obstacle.
 * aDoFirstValue if false, skip first value since it is the same as last value of last measurement in continuous mode.
 *
 * Wall detection:
 * If 2 or 3 adjacent values are quite short and the surrounding values are quite far,
 * then assume a wall which cannot reflect the pulse for the surrounding values.
 *
 * return true if display of values is managed by function itself
 */
bool fillForwardDistancesInfo(bool aShowValues, bool aDoFirstValue) {

    color16_t tColor;

// Values for forward scanning
    // Quick hack for scanning from 10 to 170 degree to avoid to detect my own wheels
//    uint8_t tActualDegrees = 0;
//    int8_t tDegreeIncrement = DEGREES_PER_STEP;
    uint8_t tActualDegrees = 10;
    int8_t tDegreeIncrement = 18;
    int8_t tIndex = 0;
    int8_t tIndexDelta = 1;
    if (sLastServoAngleInDegrees >= 170) {
// values for backward scanning
//        tActualDegrees = 180;
        tActualDegrees = 170;
        tDegreeIncrement = -(tDegreeIncrement);
        tIndex = STEPS_PER_180_DEGREES;
        tIndexDelta = -1;
    }
    if (!aDoFirstValue) {
// skip first value, since it is equal to last value of last measurement
        tIndex += tIndexDelta;
        tActualDegrees += tDegreeIncrement;
    }

    while (tIndex >= 0 && tIndex < NUMBER_OF_DISTANCES) {
        /*
         * rotate servo, wait and get distance
         */
        /*
         * compensate (set target to more degrees) for fast servo speed
         * Reasonable value is between 2 and 3 at 20 degrees and tWaitDelayforServo = tDeltaDegrees * 5
         * Reasonable value is between 10 and 20 degrees and tWaitDelayforServo = tDeltaDegrees * 4 => avoid it
         */
        US_ServoWriteAndDelay(tActualDegrees + 3, true);

        unsigned int tDistance = getUSDistanceAsCentiMeterWithCentimeterTimeout(US_TIMEOUT_CENTIMETER);

        if (((tIndex == INDEX_FORWARD_1 || tIndex == INDEX_FORWARD_2) && tDistance <= sCountPerScan)
                || (!sRunAutonomousDrive)) {
            /*
             * Emergency stop
             */
            RobotCar.stopCar();
        }

        if (aShowValues) {
            /*
             * Determine color
             */
            tColor = COLOR_ORANGE;
            if (tDistance >= US_TIMEOUT_CENTIMETER || tDistance > sCountPerScan) {
                tColor = COLOR_GREEN;
            } else if (tDistance < sCentimeterPerScan) {
                tColor = COLOR_RED;
            }

            /*
             * Clear old and draw new line
             */
            BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y,
                    sForwardDistancesInfo.RawDistancesArray[tIndex], tActualDegrees, COLOR_WHITE, 3);
            BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tDistance, tActualDegrees, tColor,
                    3);
        }
        /*
         * Store value and search for min and max
         */
        sForwardDistancesInfo.RawDistancesArray[tIndex] = tDistance;

        tIndex += tIndexDelta;
        tActualDegrees += tDegreeIncrement;
    }
    return true;
}

/*
 * Find min and max value. Prefer the headmost value if we have more than one choice
 */
void doPostProcess() {
    unsigned int tMax = 0;
    unsigned int tMin = __UINT16_MAX__; // = 65535
    for (uint8_t i = 0; i < (NUMBER_OF_DISTANCES + 1) / 2; ++i) {
        uint8_t tDistance = sForwardDistancesInfo.ProcessedDistancesArray[i];
        uint8_t tActualIndex = i;
        for (int j = 0; j < 2; ++j) {
            if (tDistance >= tMax) {
                tMax = tDistance;
                sForwardDistancesInfo.IndexOfMaxDistance = tActualIndex;
                sForwardDistancesInfo.MaxDistance = tDistance;
            }
            if (tDistance <= tMin) {
                tMin = tDistance;
                sForwardDistancesInfo.IndexOfMinDistance = tActualIndex;
                sForwardDistancesInfo.MinDistance = tDistance;
            }
            tActualIndex = STEPS_PER_180_DEGREES - i;
            tDistance = sForwardDistancesInfo.ProcessedDistancesArray[tActualIndex];
        }
    }
}

/*
 * Assume the value of 20 and 40 degrees are distances to a wall.
 * Return the clipped distance to the wall of the vector at 0 degree.
 * By changing STEPS_PER_180_degrees it can easily adopted to other degrees values.
 *
 * aDegreeFromNeigbour: The angle of the line from endpoint 0 degrees to given endpoints
 * 0 means x values of given endpoints are the same >= wall is parallel
 * Positive means wall is more ore less in front, to avoid we must turn positive angle
 * 90 means y values are the same =>  wall is in front
 * Negative means we are heading away from wall
 */
uint8_t computeNeigbourValue(uint8_t a20DegreeValue, uint8_t a40DegreeValue, uint8_t aClipValue, int8_t * aDegreeFromNeigbour) {
// assume actual = 40 Degree
    float tYat40degrees = sin((PI / STEPS_PER_180_DEGREES) * 2) * a40DegreeValue; // 40 Degree
    float tYat20degrees = sin(PI / STEPS_PER_180_DEGREES) * a20DegreeValue; // 20 Degree

//    char tStringBuffer[] = "A=_______ L=_______";
//    dtostrf(tY40Degree, 7, 2, &tStringBuffer[2]);
//    tStringBuffer[9] = ' ';
//    dtostrf(tY20Degree, 7, 2, &tStringBuffer[12]);
//    BlueDisplay1.debugMessage(tStringBuffer);

    uint8_t tZeroDegrees = aClipValue;

    /*
     * if tY40degrees == tY20degrees the tInvGradient is infinite (distance at 0 is infinite)
     */
    if (tYat40degrees > tYat20degrees) {
        float tXat40degrees = cos((PI / STEPS_PER_180_DEGREES) * 2) * a40DegreeValue; // 40 Degree
        float tXat20degrees = cos(PI / STEPS_PER_180_DEGREES) * a20DegreeValue; // 20 Degree

//        dtostrf(tX40Degree, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tX20Degree, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

//      Example for 90 and 60 degrees (since we have no other ASCII graphic symbols)
//      In this function we have 40 and 20 degrees and compute 0 degrees!
//          90 degrees value
//          |\   60 degrees value
//          | /\   \==wall
//          |/____\ 0 degrees value to be computed
        /*
         * InvGradient line represents the wall
         * if tX20degrees > tX40degrees InvGradient is negative => X0 value is bigger than X20 one / right wall is in front if we look in 90 degrees direction
         * if tX20degrees == tX40degrees InvGradient is 0 / wall is parallel right / 0 degree
         * if tX20degrees < tX40degrees InvGradient is positive / right wall is behind / degrees is negative (from direction front which is 90 degrees)
         */
        float tInvGradient = (tXat40degrees - tXat20degrees) / (tYat40degrees - tYat20degrees);
        float tXatZeroDegrees = tXat20degrees - (tInvGradient * tYat20degrees);
        *aDegreeFromNeigbour = -(atan(tInvGradient) * RAD_TO_DEG);
//        tStringBuffer[0] = 'G';
//        tStringBuffer[10] = 'B';
//        dtostrf(tInvGradient, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tXZeroDegree, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

        if (tXatZeroDegrees < 255) {
            tZeroDegrees = tXatZeroDegrees + 0.5;
            if (tZeroDegrees > aClipValue) {
                tZeroDegrees = aClipValue;
            }
        }
    }
    return tZeroDegrees;
}

/*
 * The Problem of the ultrasonic values is, that you can only detect a wall with the ultrasonic sensor if the angle of the wall relative to sensor axis is approximately between 70 and 110 degree.
 * For other angels the reflected ultrasonic beam can not not reach the receiver which leads to unrealistic great distances.
 *
 * Therefore I take samples every 20 degrees and if I get 2 adjacent short (<DISTANCE_FOR_WALL_DETECT) distances, I assume a wall determined by these 2 samples.
 * The (invalid) values 20 degrees right and left of these samples are then extrapolated by computeNeigbourValue().
 *
 */
void doWallDetection(bool aShowValues) {
    uint8_t tTempDistancesArray[NUMBER_OF_DISTANCES];
    /*
     * First copy all raw values
     */
    memcpy(tTempDistancesArray, sForwardDistancesInfo.RawDistancesArray, NUMBER_OF_DISTANCES);
    uint8_t tLastValue = tTempDistancesArray[0];
    uint8_t tActualValue = tTempDistancesArray[1];
    uint8_t tNextValue;
    uint8_t tActualDegrees = 2 * DEGREES_PER_STEP;
    int8_t tDegreeFromNeigbour;
    sForwardDistancesInfo.WallRightAngleDegree = 0;
    sForwardDistancesInfo.WallLeftAngleDegree = 0;

    /*
     * check values at i and i-1 and adjust value at i+1
     * i is index of ActualValue
     */
    for (uint8_t i = 1; i < STEPS_PER_180_DEGREES; ++i) {
        tNextValue = tTempDistancesArray[i + 1];
        if (tLastValue < sCountPerScan && tActualValue < sCountPerScan) {
            /*
             * Wall detected -> adjust adjacent values
             */

            // use computeNeigbourValue the other way round
            // i.e. put 20 degrees to 40 degrees parameter and vice versa in order to take the 0 degrees value as the 60 degrees one
            uint8_t tNextValueComputed = computeNeigbourValue(tActualValue, tLastValue, US_TIMEOUT_CENTIMETER,
                    &tDegreeFromNeigbour);
            if (tNextValue > tNextValueComputed + 5) {
//                BlueDisplay1.debug("i=", i);
//                BlueDisplay1.debug("fwddegrees=", tDegreeFromNeigbour);
                // degrees of computed value - returned wall degrees seen from (degrees of computed value)
                int tWallForwardDegrees = ((i + 1) * DEGREES_PER_STEP) - tDegreeFromNeigbour;
//                BlueDisplay1.debug("wall forw degrees=", tWallForwardDegrees);
                if (tWallForwardDegrees <= 90) {
                    // wall at right
                    sForwardDistancesInfo.WallRightAngleDegree = tWallForwardDegrees;
                } else {
                    // wall at left
                    sForwardDistancesInfo.WallLeftAngleDegree = 180 - tWallForwardDegrees;
                }

                //Adjust and draw next value if original value is greater
                tTempDistancesArray[i + 1] = tNextValueComputed;
                tNextValue = tNextValueComputed;
                if (aShowValues) {
                    BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextValueComputed,
                            tActualDegrees,
                            COLOR_BLACK, 1);
                }
            }
        }
        tLastValue = tActualValue;
        tActualValue = tNextValue;
        tActualDegrees += DEGREES_PER_STEP;
    }

    /*
     * Go backwards through the array
     */
    memcpy(sForwardDistancesInfo.ProcessedDistancesArray, tTempDistancesArray, NUMBER_OF_DISTANCES);

    tLastValue = tTempDistancesArray[STEPS_PER_180_DEGREES];
    tActualValue = tTempDistancesArray[STEPS_PER_180_DEGREES - 1];
    tActualDegrees = 180 - (2 * DEGREES_PER_STEP);

    /*
     * check values at i and i+1 and adjust value at i-1
     */
    for (uint8_t i = STEPS_PER_180_DEGREES - 1; i > 0; --i) {
        tNextValue = tTempDistancesArray[i - 1];

// Do it only if none of the 3 values are processed before
        if (tTempDistancesArray[i + 1] == sForwardDistancesInfo.RawDistancesArray[i + 1]
                && tTempDistancesArray[i] == sForwardDistancesInfo.RawDistancesArray[i]
                && tNextValue == sForwardDistancesInfo.RawDistancesArray[i - 1]) {

            /*
             * check values at i+1 and i and adjust value at i-11
             */
            if (tLastValue < sCountPerScan && tActualValue < sCountPerScan) {
                /*
                 * Wall detected -> adjust adjacent values
                 */
                uint8_t tNextValueComputed = computeNeigbourValue(tActualValue, tLastValue, US_TIMEOUT_CENTIMETER,
                        &tDegreeFromNeigbour);
                if (tNextValue > tNextValueComputed + 5) {
//                    BlueDisplay1.debug("i=", i);
//                    BlueDisplay1.debug("backdegrees=", tDegreeFromNeigbour);
                    // only left and front
                    int tWallBackwardDegrees = (180 - ((i - 1) * DEGREES_PER_STEP)) - tDegreeFromNeigbour;
//                    BlueDisplay1.debug("wall back degrees=", tWallBackwardDegrees);
                    if (tWallBackwardDegrees <= 90) {
                        // wall at left - overwrite only if greater
                        if (sForwardDistancesInfo.WallLeftAngleDegree < tWallBackwardDegrees) {
                            sForwardDistancesInfo.WallLeftAngleDegree = tWallBackwardDegrees;
                        }
                    } else if (sForwardDistancesInfo.WallRightAngleDegree < (180 - tWallBackwardDegrees)) {
                        // wall at right - overwrite only if greater
                        sForwardDistancesInfo.WallRightAngleDegree = 180 - tWallBackwardDegrees;
                        BlueDisplay1.debug("sWallRightDegree=", sForwardDistancesInfo.WallRightAngleDegree);

                    }
                    //Adjust and draw next value if original value is greater
                    sForwardDistancesInfo.ProcessedDistancesArray[i - 1] = tNextValueComputed;
                    tNextValue = tNextValueComputed;
                    if (aShowValues) {
                        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextValueComputed,
                                tActualDegrees, COLOR_BLACK, 1);
                    }
                }
            }
        }
        tLastValue = tActualValue;
        tActualValue = tNextValue;
        tActualDegrees -= DEGREES_PER_STEP;

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
    if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_1] > sCountPerScan
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_2] > sCountPerScan) {
        /*
         * Free ahead, check if our side is near to the wall and make corrections
         */
        if (sForwardDistancesInfo.WallRightAngleDegree != 0 || sForwardDistancesInfo.WallLeftAngleDegree != 0) {
            /*
             * Wall detected
             */
            if (sForwardDistancesInfo.WallRightAngleDegree > sForwardDistancesInfo.WallLeftAngleDegree) {
                /*
                 * Wall at right => turn left
                 */
                tDegreeToTurn = sForwardDistancesInfo.WallRightAngleDegree;
            } else {
                /*
                 * Wall at left => turn right
                 */
                tDegreeToTurn = -sForwardDistancesInfo.WallLeftAngleDegree;
            }

        }
    } else {
        if (sForwardDistancesInfo.WallRightAngleDegree != 0 || sForwardDistancesInfo.WallLeftAngleDegree != 0) {
            /*
             * Wall detected
             */
            if (sForwardDistancesInfo.WallRightAngleDegree > sForwardDistancesInfo.WallLeftAngleDegree) {
                /*
                 * Wall at right => turn left
                 */
                tDegreeToTurn = sForwardDistancesInfo.WallRightAngleDegree;
            } else {
                /*
                 * Wall at left => turn right
                 */
                tDegreeToTurn = -sForwardDistancesInfo.WallLeftAngleDegree;
            }
        } else {
            /*
             * Not free ahead, must turn, check if another forward direction is suitable
             */
            if (sForwardDistancesInfo.MaxDistance > sCountPerScan) {
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
void driveAutonomousOneStep(bool (*aFillForwardDistancesInfoFunction)(bool, bool), int (*aCollisionDetectionFunction)()) {

    /*
     * 1. Check for step conditions if step should happen
     */
    if (sStepMode == MODE_CONTINUOUS || (sStepMode == MODE_SINGLE_STEP && sDoStep)
            || (sStepMode == MODE_STEP_TO_NEXT_TURN && (!RobotCar.isStopped() || sDoStep))) {
        /*
         * Do one step
         */
        bool tMovementJustStarted = sDoStep; // tMovementJustStarted is needed for speeding up US scanning
        sDoStep = false; // Now it can be set again by GUI

        /*
         * Handle both step modes here
         */
        int sLastDisplayedDegreeToTurn = sNextDegreesToTurn;
        if (sStepMode == MODE_SINGLE_STEP) {
            /*
             * SINGLE_STEP -> optional turn and go fixed distance
             */
            if (sNextDegreesToTurn == GO_BACK_AND_SCAN_AGAIN) {
                RobotCar.goDistanceCentimeter(-10, &loopGUI);
            } else {
                RobotCar.rotateCar(sNextDegreesToTurn, sTurnMode);
                sLastDegreesTurned = sNextDegreesToTurn;
                sNextDegreesToTurn = 0;
                RobotCar.goDistanceCentimeter(CENTIMETER_PER_RIDE, &loopGUI);
            }
        } else
        /*
         * MODE_STEP_TO_NEXT_TURN or MODE_CONTINUOUS: rotation requested -> rotate and start again
         */
        if (RobotCar.isStopped()) {
            if (sNextDegreesToTurn == GO_BACK_AND_SCAN_AGAIN) {
                RobotCar.goDistanceCentimeter(-10, &loopGUI);
            } else {
                RobotCar.rotateCar(sNextDegreesToTurn, sTurnMode);
                // wait to really stop after turning
                delay(100);
                sLastDegreesTurned = sNextDegreesToTurn;
                sNextDegreesToTurn = 0;
                RobotCar.startAndWaitForFullSpeed();
                tMovementJustStarted = true;
//            delay(100);
            }
        }

        /*
         * Here car has moved
         */

        bool tActualPageIsAutomaticControl = (sActualPage == PAGE_AUTOMATIC_CONTROL);
        if (tActualPageIsAutomaticControl && ((sLastDisplayedDegreeToTurn + 10) % DEGREES_PER_STEP) != 0) {
            /*
             * Clear old decision marker by redrawing it with a white line if not overlapped with a distance bar at 10, 30, 50, 70, 90 degree
             */
            drawCollisionDecision(sLastDisplayedDegreeToTurn, CENTIMETER_PER_RIDE, true);
        }

        uint16_t tStartCount = leftEncoderMotor.DistanceCount;

        /*
         * The magic happens HERE
         */
        bool tInfoWasProcessed = aFillForwardDistancesInfoFunction(tActualPageIsAutomaticControl, tMovementJustStarted);
        doWallDetection(tActualPageIsAutomaticControl);
        sNextDegreesToTurn = aCollisionDetectionFunction();

        /*
         * compute distance driven for one 180 degrees scan
         */
        if (!RobotCar.isStopped()) {
            /*
             * No emergency stop here => distance is valid
             */
            sCountPerScan = leftEncoderMotor.DistanceCount - tStartCount;
            sCentimeterPerScan = sCountPerScan / 2;
            if (tActualPageIsAutomaticControl) {
                char tStringBuffer[6];
                sprintf_P(tStringBuffer, PSTR("%2d%s"), sCentimeterPerScan, "cm");
                BlueDisplay1.drawText(0, BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND, tStringBuffer, TEXT_SIZE_11, COLOR_BLACK,
                COLOR_WHITE);
            }
        }

        /*
         * Show distance info if not already done
         */
        if (!tInfoWasProcessed && tActualPageIsAutomaticControl) {
            drawForwardDistancesInfos();
        }
        drawCollisionDecision(sNextDegreesToTurn, sCentimeterPerScan, false);

        /*
         *
         */
        if (sNextDegreesToTurn != 0 || sStepMode == MODE_SINGLE_STEP) {
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
             * just continue => overwrite last path element with actual riding distance and try to synchronize motors
             */
            insertToPath(rightEncoderMotor.DistanceCount, sLastDegreesTurned, false);
            rightEncoderMotor.synchronizeMotor(&leftEncoderMotor, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
        }
        if (sActualPage == PAGE_SHOW_PATH) {
            drawPathInfoPage();
        }
    }

}

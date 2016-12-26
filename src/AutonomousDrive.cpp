/*
 * AutonomousDrive.cpp
 *
 * Contains:
 * fillForwardDistancesInfoPro(): Acquisition of 180 degree distances by ultrasonic sensor and servo
 * doWallDetection(): Enhancement of acquired data because of lack of detecting flat surfaces by US at angels out of 70 to 110 degree.
 * doCollisionDetectionPro(): decision where to turn in dependency of the acquired distances.
 * driveAutonomous(): The loop which handles the start/stop, single step and path output functionality.
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

#include <AutonomousDrive.h>
#include "RobotCar.h"
#include "RobotCarGui.h"

#include "EncoderMotorControl.h"
#include "ArminsUtils.h"
#include <stdlib.h> // for dtostrf()
//

ForwardDistancesInfoStruct ForwardDistancesInfo;
uint8_t sLastServoAngleInDegree; // 0 - 180 needed for optimized delay for servo repositioning

// Storage for turning decision especially for single step mode
int sNextDegreeToTurn = 0;
// Storage of last turning for insertToPath()
int sLastDegreeTurned = 0;

// 0 degree => wall parallel to side of car. 90 degree => wall in front of car. Degree of wall -> degree to turn.
int8_t sWallRightDegree = 0;
int8_t sWallLeftDegree = 0;

uint8_t sCountPerScan = CENTIMETER_PER_RIDE_PRO * 2;
uint8_t sCentimeterPerScan = CENTIMETER_PER_RIDE_PRO;

/*
 * sets also sLastServoAngleInDegree to enable optimized servo movement and delays
 * SG90 Micro Servo has reached its end position if the current (200 mA) is low for more than 11 to 14 ms
 */

void ServoWrite(uint8_t aValueDegree, bool doDelay) {

    if (aValueDegree > 220) {
        // handle underflow
        aValueDegree = 0;
    } else if (aValueDegree > 180) {
        // handle underflow
        aValueDegree = 180;
    }
    uint8_t tDeltaDegrees = abs(sLastServoAngleInDegree - aValueDegree);
    sLastServoAngleInDegree = aValueDegree;
    if (myCar.is2WDCar) {
        // My servo on the 2WD car is top down and therefore inverted
        aValueDegree = 180 - aValueDegree;
    }
    digitalWrite(DEBUG1_PIN, HIGH);
    setSimpleServoPulsePin10(aValueDegree);
    if (tDeltaDegrees == 0) {
        return;
    }
    if (doDelay) {
        myCar.rightMotorControl.syncronizeMotor(&myCar.leftMotorControl, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
        loopGUI();
        // Datasheet says: SG90 Micro Servo needs 100 millis per 60 degree angle => 300 ms per 180
        // I measured: SG90 Micro Servo needs 400 per 180 degree and 400 per 2*90 degree, but 540 millis per 9*20 degree
        // 60-80 ms for 20 degrees

//        // wait at least 5 ms for the servo to receive signal
//        delay(SERVO_INITIAL_DELAY);
//        digitalWrite(DEBUG1_PIN, LOW);

        // factor 8 gives a fairly reproducible result, factor 4 is a bit too fast
        uint16_t tWaitDelayforServo = tDeltaDegrees * 5;

// did nor really work :-(
//        uint16_t tCurrent = getADCValue(SERVO_CURRENT_IN_CHANNEL, INTERNAL);
//        BlueDisplay1.debug("Current=", tCurrent);
//
//        if (tCurrent > SERVO_CURRENT_LOW_THRESHOLD) {
//            // Servo current detected use it for determine the stop of servo
//            long tMillisOfLoopStart = millis();
//            long tStartMillisOfCurrentLow = tMillisOfLoopStart;
//            long tMillis;
//            do {
//                tMillis = millis();
//                if (getADCValue(SERVO_CURRENT_IN_CHANNEL, INTERNAL) > SERVO_CURRENT_LOW_THRESHOLD) {
//                    // reset counter and continue
//                    tStartMillisOfCurrentLow = tMillis;
//                    continue;
//                } else {
//                    if (millis() - tStartMillisOfCurrentLow > SERVO_CURRENT_LOW_MILLIS_FOR_SERVO_STOPPED) {
//                        // read low for more than 12 millis => assume servo has reached its position
//                        BlueDisplay1.debug("Time=", (uint16_t) (tMillis - tMillisOfLoopStart));
//                        break;
//                    }
//                }
//            } while (tMillis - tMillisOfLoopStart < tWaitDelayforServo);
//        } else {
        delay(tWaitDelayforServo);

    }
}

/*
 * Clear drawing area
 */
void clearPrintedForwardDistancesInfos() {
    BlueDisplay1.fillRectRel(US_DISTANCE_MAP_ORIGIN_X - US_DISTANCE_MAP_WIDTH_HALF,
    US_DISTANCE_MAP_ORIGIN_Y - US_DISTANCE_MAP_HEIGHT, US_DISTANCE_MAP_WIDTH_HALF * 2, US_DISTANCE_MAP_HEIGHT + 2, COLOR_WHITE);
}

/*
 * Draw values of ActualDistancesArray as vectors
 */
void drawForwardDistancesInfos(ForwardDistancesInfoStruct* aForwardDistancesInfo) {
    Color_t tColor;
    uint8_t tActualDegree = 0;
    /*
     * Clear drawing area
     */
    clearPrintedForwardDistancesInfos();
    for (int i = 0; i < NUMBER_OF_DISTANCES; ++i) {
        /*
         * Determine color
         */
        uint8_t tDistance = aForwardDistancesInfo->RawDistancesArray[i];
        tColor = COLOR_ORANGE;
        if (tDistance >= US_TIMEOUT_CENTIMETER_PRO) {
            tDistance = US_TIMEOUT_CENTIMETER_PRO;
            tColor = COLOR_GREEN;
        }
        if (tDistance > sCountPerScan) {
            tColor = COLOR_GREEN;
        } else if (tDistance < sCentimeterPerScan) {
            tColor = COLOR_RED;
        }

        /*
         * Draw line
         */
        BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tDistance, tActualDegree, tColor, 3);
        tActualDegree += DEGREE_PER_STEP;
    }
    doWallDetection(aForwardDistancesInfo, true);
}

void drawCollisionDecision(int aDegreeToTurn, uint8_t aLengthOfVector, bool aDoClear) {
    if (sActualPage == PAGE_AUTOMATIC_CONTROL) {
        Color_t tColor = COLOR_YELLOW;
        int tDegreeToDisplay = aDegreeToTurn;
        if (tDegreeToDisplay == 180) {
            tColor = COLOR_MAGENTA;
            tDegreeToDisplay = 0;
        }
        if (aDoClear) {
            tColor = COLOR_WHITE;
        }
        BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, aLengthOfVector, tDegreeToDisplay + 90,
                tColor);
        if (!aDoClear) {
            sprintf_P(sStringBuffer, PSTR("wall%4d\xB0 rotation: %3d\xB0 wall%4d\xB0"), sWallLeftDegree, aDegreeToTurn,
                    sWallRightDegree);
            BlueDisplay1.drawText(US_DISTANCE_MAP_ORIGIN_X - US_DISTANCE_MAP_WIDTH_HALF, US_DISTANCE_MAP_ORIGIN_Y + TEXT_SIZE_11,
                    sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
        }
    }
}
/*
 * Get 7 distances starting at 0 degree (right) increasing by 30 degree up to 180 degree (left)
 * aDoFirstValue if false, skip first value since it is the same as last value of last measurement in continuous mode.
 *
 * Wall detection:
 * If 2 or 3 adjacent values are quite short and the surrounding values are quite far,
 * then assume a wall which cannot reflect the pulse for the surrounding values.
 *
 * return true if display of values is managed by function itself
 */
bool fillForwardDistancesInfo(ForwardDistancesInfoStruct* aForwardDistancesInfo, bool aShowValues,
bool aDoFirstValue) {

    Color_t tColor;

// Values for forward scanning
    uint8_t tActualDegree = 0;
    int8_t tDegreeIncrement = DEGREE_PER_STEP;
    int8_t tIndex = 0;
    int8_t tIndexDelta = 1;
    if (sLastServoAngleInDegree >= 180) {
// values for backward scanning
        tActualDegree = 180;
        tDegreeIncrement = -(DEGREE_PER_STEP);
        tIndex = STEPS_PER_180_DEGREE;
        tIndexDelta = -1;
    }
    if (!aDoFirstValue) {
// skip first value, since it is equal to last value of last measurement
        tIndex += tIndexDelta;
        tActualDegree += tDegreeIncrement;
    }

    while (tIndex >= 0 && tIndex < NUMBER_OF_DISTANCES) {
        /*
         * rotate servo, wait and get distance
         */
        /*
         * compensate (set target to more degree) for fast servo speed
         * Reasonable value is between 2 and 3 at 20 degree and tWaitDelayforServo = tDeltaDegrees * 5
         * Reasonable value is between 10 and 20 degree and tWaitDelayforServo = tDeltaDegrees * 4 => avoid it
         */
        ServoWrite(tActualDegree + 3, true);

        unsigned int tDistance = getUSDistanceAsCentiMeterWithCentimeterTimeout(US_TIMEOUT_CENTIMETER_PRO);

        if (((tIndex == INDEX_FORWARD_1 || tIndex == INDEX_FORWARD_2) && tDistance <= sCountPerScan)
                || (!(sRunAutonomousDrive || sRunOwnTest))) {
            /*
             * Emergency stop
             */
            myCar.stopCar();
        }

        if (aShowValues) {
            /*
             * Determine color
             */
            tColor = COLOR_ORANGE;
            if (tDistance == US_TIMEOUT_CENTIMETER_PRO || tDistance > sCountPerScan) {
                tColor = COLOR_GREEN;
            } else if (tDistance < sCentimeterPerScan) {
                tColor = COLOR_RED;
            }

            /*
             * Clear old and draw new line
             */
            BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y,
                    aForwardDistancesInfo->RawDistancesArray[tIndex], tActualDegree, COLOR_WHITE, 3);
            BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tDistance, tActualDegree, tColor, 3);
        }
        /*
         * Store value and search for min and max
         */
        aForwardDistancesInfo->RawDistancesArray[tIndex] = tDistance;

        tIndex += tIndexDelta;
        tActualDegree += tDegreeIncrement;
    }
    return true;
}

/*
 * Find min and max value. Prefer the headmost value if we have more than one choice
 */
void doPostProcess(ForwardDistancesInfoStruct* aForwardDistancesInfo) {
    unsigned int tMax = 0;
    unsigned int tMin = __UINT16_MAX__; // = 65535
    for (uint8_t i = 0; i < (NUMBER_OF_DISTANCES + 1) / 2; ++i) {
        uint8_t tDistance = aForwardDistancesInfo->ProcessedDistancesArray[i];
        uint8_t tActualIndex = i;
        for (int j = 0; j < 2; ++j) {
            if (tDistance >= tMax) {
                tMax = tDistance;
                aForwardDistancesInfo->IndexOfMaxDistance = tActualIndex;
                aForwardDistancesInfo->MaxDistance = tDistance;
            }
            if (tDistance <= tMin) {
                tMin = tDistance;
                aForwardDistancesInfo->IndexOfMinDistance = tActualIndex;
                aForwardDistancesInfo->MinDistance = tDistance;
            }
            tActualIndex = STEPS_PER_180_DEGREE - i;
            tDistance = aForwardDistancesInfo->ProcessedDistancesArray[tActualIndex];
        }
    }
}

/*
 * Assume the value of 20 and 40 degree are distances to a wall.
 * Return the clipped distance to the wall of the vector at 0 degree.
 * By changing STEPS_PER_180_DEGREE it can easily adopted to other degree values.
 *
 * aDegreeFromNeigbour: The angle of the line from endpoint 0 degree to given endpoints
 * 0 means x values of given endpoints are the same >= wall is parallel
 * Positive means wall is more ore less in front, to avoid we must turn positive angle
 * 90 means y values are the same =>  wall is in front
 * Negative means we are heading away from wall
 */
uint8_t computeNeigbourValue(uint8_t a20DegreeValue, uint8_t a40DegreeValue, uint8_t aClipValue, int8_t * aDegreeFromNeigbour) {
// assume actual = 40 Degree
    float tYat40Degree = sin((PI / STEPS_PER_180_DEGREE) * 2) * a40DegreeValue; // 40 Degree
    float tYat20Degree = sin(PI / STEPS_PER_180_DEGREE) * a20DegreeValue; // 20 Degree

//    char tStringBuffer[] = "A=_______ L=_______";
//    dtostrf(tY40Degree, 7, 2, &tStringBuffer[2]);
//    tStringBuffer[9] = ' ';
//    dtostrf(tY20Degree, 7, 2, &tStringBuffer[12]);
//    BlueDisplay1.debugMessage(tStringBuffer);

    uint8_t tZeroDegree = aClipValue;

    /*
     * if tY40Degree == tY20Degree the tInvGradient is infinite (distance at 0 is infinite)
     */
    if (tYat40Degree > tYat20Degree) {
        float tXat40Degree = cos((PI / STEPS_PER_180_DEGREE) * 2) * a40DegreeValue; // 40 Degree
        float tXat20Degree = cos(PI / STEPS_PER_180_DEGREE) * a20DegreeValue; // 20 Degree

//        dtostrf(tX40Degree, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tX20Degree, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

//      Example for 90 and 60 degree (since we have no other ASCII graphic symbols)
//          90 degree value
//          |\   60 degree value
//          | /\   \==wall
//          |/____\ 0 degree value to be computed
        /*
         * InvGradient line represents the wall
         * if tX20Degree > tX40Degree InvGradient is negative => X0 value is bigger than X20 one / right wall is in front if we look in 90 degree direction
         * if tX20Degree == tX40Degree InvGradient is 0 / wall is parallel right / 0 degree
         * if tX20Degree < tX40Degree InvGradient is positive / right wall is behind / degree is negative (from direction front which is 90 degree)
         */
        float tInvGradient = (tXat40Degree - tXat20Degree) / (tYat40Degree - tYat20Degree);
        float tXatZeroDegree = tXat20Degree - (tInvGradient * tYat20Degree);
        *aDegreeFromNeigbour = -(atan(tInvGradient) * RAD_TO_DEG);
//        tStringBuffer[0] = 'G';
//        tStringBuffer[10] = 'B';
//        dtostrf(tInvGradient, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tXZeroDegree, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

        if (tXatZeroDegree < 255) {
            tZeroDegree = tXatZeroDegree + 0.5;
            if (tZeroDegree > aClipValue) {
                tZeroDegree = aClipValue;
            }
        }
    }
    return tZeroDegree;
}

/*
 * The Problem of the ultrasonic values is, that you can only detect a wall with the ultrasonic sensor if the angle of the wall relative to sensor axis is approximately between 70 and 110 degree.
 * For other angels the reflected ultrasonic beam can not not reach the receiver which leads to unrealistic great distances.
 *
 * Therefore I take samples every 20 degree and if I get 2 adjacent short (<DISTANCE_FOR_WALL_DETECT) distances, I assume a wall determined by these 2 samples.
 * The (invalid) values 20 degrees right and left of these samples are then extrapolated by computeNeigbourValue().
 *
 */
void doWallDetection(ForwardDistancesInfoStruct* aForwardDistancesInfo, bool aShowValues) {
    uint8_t tTempDistancesArray[NUMBER_OF_DISTANCES];
    /*
     * First copy all raw values
     */
    memcpy(tTempDistancesArray, aForwardDistancesInfo->RawDistancesArray, NUMBER_OF_DISTANCES);
    uint8_t tLastValue = tTempDistancesArray[0];
    uint8_t tActualValue = tTempDistancesArray[1];
    uint8_t tNextValue;
    uint8_t tActualDegree = 2 * DEGREE_PER_STEP;
    int8_t tDegreeFromNeigbour;
    sWallRightDegree = 0;
    sWallLeftDegree = 0;

    /*
     * check values at i and i-1 and adjust value at i+1
     * i is index of ActualValue
     */
    for (uint8_t i = 1; i < STEPS_PER_180_DEGREE; ++i) {
        tNextValue = tTempDistancesArray[i + 1];
        if (tLastValue < sCountPerScan && tActualValue < sCountPerScan) {
            /*
             * Wall detected -> adjust adjacent values
             */

            // use computeNeigbourValue the other way round
            // i.e. put 20 degree to 40 degree parameter and vice versa in order to take the 0 degree value as the 60 degree one
            uint8_t tNextValueComputed = computeNeigbourValue(tActualValue, tLastValue, US_TIMEOUT_CENTIMETER_PRO,
                    &tDegreeFromNeigbour);
            if (tNextValue > tNextValueComputed + 5) {
                BlueDisplay1.debug("i=", i);
                BlueDisplay1.debug("fwddegree=", tDegreeFromNeigbour);
                // degree of computed value - returned wall degree seen from (degree of computed value)
                int tWallForwardDegree = ((i + 1) * DEGREE_PER_STEP) - tDegreeFromNeigbour;
                BlueDisplay1.debug("wall forw degree=", tWallForwardDegree);
                if (tWallForwardDegree <= 90) {
                    // wall at right
                    sWallRightDegree = tWallForwardDegree;
                } else {
                    // wall at left
                    sWallLeftDegree = 180 - tWallForwardDegree;
                }

                //Adjust and draw next value if original value is greater
                tTempDistancesArray[i + 1] = tNextValueComputed;
                tNextValue = tNextValueComputed;
                if (aShowValues) {
                    BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextValueComputed,
                            tActualDegree,
                            COLOR_BLACK, 1);
                }
            }
        }
        tLastValue = tActualValue;
        tActualValue = tNextValue;
        tActualDegree += DEGREE_PER_STEP;
    }

    /*
     * Go backwards through the array
     */
    memcpy(aForwardDistancesInfo->ProcessedDistancesArray, tTempDistancesArray, NUMBER_OF_DISTANCES);

    tLastValue = tTempDistancesArray[STEPS_PER_180_DEGREE];
    tActualValue = tTempDistancesArray[STEPS_PER_180_DEGREE - 1];
    tActualDegree = 180 - (2 * DEGREE_PER_STEP);

    /*
     * check values at i and i+1 and adjust value at i-1
     */
    for (uint8_t i = STEPS_PER_180_DEGREE - 1; i > 0; --i) {
        tNextValue = tTempDistancesArray[i - 1];

// Do it only if none of the 3 values are processed before
        if (tTempDistancesArray[i + 1] == aForwardDistancesInfo->RawDistancesArray[i + 1]
                && tTempDistancesArray[i] == aForwardDistancesInfo->RawDistancesArray[i]
                && tNextValue == aForwardDistancesInfo->RawDistancesArray[i - 1]) {

            /*
             * check values at i+1 and i and adjust value at i-11
             */
            if (tLastValue < sCountPerScan && tActualValue < sCountPerScan) {
                /*
                 * Wall detected -> adjust adjacent values
                 */
                uint8_t tNextValueComputed = computeNeigbourValue(tActualValue, tLastValue, US_TIMEOUT_CENTIMETER_PRO,
                        &tDegreeFromNeigbour);
                if (tNextValue > tNextValueComputed + 5) {
                    BlueDisplay1.debug("i=", i);
                    BlueDisplay1.debug("backdegree=", tDegreeFromNeigbour);
                    // only left and front
                    int tWallBackwardDegree = (180 - ((i - 1) * DEGREE_PER_STEP)) - tDegreeFromNeigbour;
                    BlueDisplay1.debug("wall back degree=", tWallBackwardDegree);
                    if (tWallBackwardDegree <= 90) {
                        // wall at left - overwrite only if greater
                        if (sWallLeftDegree < tWallBackwardDegree) {
                            sWallLeftDegree = tWallBackwardDegree;
                        }
                    } else if (sWallRightDegree < (180 - tWallBackwardDegree)) {
                        // wall at right - overwrite only if greater
                        sWallRightDegree = 180 - tWallBackwardDegree;
                        BlueDisplay1.debug("sWallRightDegree=", sWallRightDegree);

                    }
                    //Adjust and draw next value if original value is greater
                    aForwardDistancesInfo->ProcessedDistancesArray[i - 1] = tNextValueComputed;
                    tNextValue = tNextValueComputed;
                    if (aShowValues) {
                        BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextValueComputed,
                                tActualDegree, COLOR_BLACK, 1);
                    }
                }
            }
        }
        tLastValue = tActualValue;
        tActualValue = tNextValue;
        tActualDegree -= DEGREE_PER_STEP;

    }
    doPostProcess(aForwardDistancesInfo);
}

#define GO_BACK_AND_SCAN_AGAIN 360
/*
 * Checks distances and returns degree to turn
 * 0 -> no turn, > 0 -> turn left, < 0 -> turn right, > 360 go back, since too close to wall
 */
int doCollisionDetectionPro(ForwardDistancesInfoStruct* aForwardDistancesInfo) {
    int tDegreeToTurn = 0;
    // 5 is too low
    if (aForwardDistancesInfo->MinDistance < 7) {
        /*
         * Min Distance too small => go back and scan again
         */
        return GO_BACK_AND_SCAN_AGAIN;
    }
    /*
     * First check if free ahead
     */
    if (aForwardDistancesInfo->ProcessedDistancesArray[INDEX_FORWARD_1] > sCountPerScan
            && aForwardDistancesInfo->ProcessedDistancesArray[INDEX_FORWARD_2] > sCountPerScan) {
        /*
         * Free ahead, check if our side is near to the wall and make corrections
         */
        if (sWallRightDegree != 0 || sWallLeftDegree != 0) {
            /*
             * Wall detected
             */
            if (sWallRightDegree > sWallLeftDegree) {
                /*
                 * Wall at right => turn left
                 */
                tDegreeToTurn = sWallRightDegree;
            } else {
                /*
                 * Wall at left => turn right
                 */
                tDegreeToTurn = -sWallLeftDegree;
            }

        }
    } else {
        if (sWallRightDegree != 0 || sWallLeftDegree != 0) {
            /*
             * Wall detected
             */
            if (sWallRightDegree > sWallLeftDegree) {
                /*
                 * Wall at right => turn left
                 */
                tDegreeToTurn = sWallRightDegree;
            } else {
                /*
                 * Wall at left => turn right
                 */
                tDegreeToTurn = -sWallLeftDegree;
            }
        } else {
            /*
             * Not free ahead, must turn, check if another forward direction is suitable
             */
            if (aForwardDistancesInfo->MaxDistance > sCountPerScan) {
                /*
                 * Go to max distance
                 */
                tDegreeToTurn = aForwardDistancesInfo->IndexOfMaxDistance * DEGREE_PER_STEP - 90;
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

void driveAutonomous(bool (*afillForwardDistancesInfoFunction)(ForwardDistancesInfoStruct*, bool,
bool), int (*aCollisionDetectionFunction)(ForwardDistancesInfoStruct*)) {

    if (sStepMode == MODE_CONTINUOUS || (sStepMode == MODE_SINGLE_STEP && sDoStep)
            || (sStepMode == MODE_STEP && (!myCar.isStopped() || sDoStep))) {
        /*
         * Driving is enabled here
         */
        bool tJustStarted = sDoStep;
        sDoStep = false;

        /*
         * Handle both step modes here
         */
        int sLastDisplayedDegreeToTurn = sNextDegreeToTurn;
        if (sStepMode == MODE_SINGLE_STEP) {
            /*
             * SINGLE_STEP -> turn and go fixed distance
             */
            if (sNextDegreeToTurn == GO_BACK_AND_SCAN_AGAIN) {
                myCar.goDistanceCentimeter(-10, &loopGUI);
            } else {
                myCar.rotateCar(sNextDegreeToTurn); // TURN_IN_PLACE
                sLastDegreeTurned = sNextDegreeToTurn;
                sNextDegreeToTurn = 0;
                myCar.goDistanceCentimeter(CENTIMETER_PER_RIDE_PRO, &loopGUI);
            }
        } else if (myCar.isStopped()) {
            /*
             * MODE_SINGLE_STEP or MODE_CONTINUOUS: rotation requested -> rotate and start again
             */
            if (sNextDegreeToTurn == GO_BACK_AND_SCAN_AGAIN) {
                myCar.goDistanceCentimeter(-10, &loopGUI);
            } else {
                myCar.rotateCar(sNextDegreeToTurn, (uint8_t) TURN_BACKWARD);
                // wait to really stop after turning
                delay(100);
                sLastDegreeTurned = sNextDegreeToTurn;
                sNextDegreeToTurn = 0;
                myCar.startAndWaitForFullSpeed();
                tJustStarted = true;
//            delay(100);
            }
        }

        bool tActualPageIsAutomaticControl = (sActualPage == PAGE_AUTOMATIC_CONTROL);

        if (tActualPageIsAutomaticControl && ((sLastDisplayedDegreeToTurn + 10) % DEGREE_PER_STEP) != 0) {
            /*
             * Clear old decision marker by redrawing it with a white line if not overlapped with a distance bar at 10, 30, 50, 70, 90 degree
             */
            drawCollisionDecision(sLastDisplayedDegreeToTurn, CENTIMETER_PER_RIDE_PRO, true);
        }

        uint16_t tStartCount = myCar.leftMotorControl.DistanceCount;

        /*
         * The magic happens HERE
         */
        bool tInfoWasProcessed = afillForwardDistancesInfoFunction(&ForwardDistancesInfo, tActualPageIsAutomaticControl,
                tJustStarted);
        doWallDetection(&ForwardDistancesInfo, tActualPageIsAutomaticControl);
        sNextDegreeToTurn = aCollisionDetectionFunction(&ForwardDistancesInfo);

        /*
         * compute distance driven for one 180 degree scan
         */
        if (!myCar.isStopped()) {
            /*
             * No emergency stop here => distance is valid
             */
            sCountPerScan = myCar.leftMotorControl.DistanceCount - tStartCount;
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
            drawForwardDistancesInfos(&ForwardDistancesInfo);
        }
        drawCollisionDecision(sNextDegreeToTurn, sCentimeterPerScan, false);

        /*
         *
         */
        if (sNextDegreeToTurn != 0 || sStepMode == MODE_SINGLE_STEP) {
            /*
             * Stop if rotation requested or single step => insert / update last ride in path
             */
            myCar.stopCar();
            if (sStepMode == MODE_SINGLE_STEP) {
                insertToPath(CENTIMETER_PER_RIDE_PRO * 2, sLastDegreeTurned, true);
            } else {
                // add last driven distance to path
                insertToPath(myCar.rightMotorControl.LastRideDistanceCount, sLastDegreeTurned, true);
            }
        } else {
            /*
             * just continue => overwrite last path element with actual riding distance and try to synchronize motors
             */
            insertToPath(myCar.rightMotorControl.DistanceCount, sLastDegreeTurned, false);
            myCar.rightMotorControl.syncronizeMotor(&myCar.leftMotorControl, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
        }
        if (sActualPage == PAGE_SHOW_PATH) {
            DrawPath();
        }
    }

}

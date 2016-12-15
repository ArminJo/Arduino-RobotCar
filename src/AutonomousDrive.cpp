/*
 * AutonomousDrive.cpp
 *
 * Collision detection and autonomous driving functions
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
uint8_t sLastServoAngleInDegree; // needed for optimized delay for servo repositioning
int sDegreeToTurn = 0;
int sLastDegreeToTurn = 0;
uint16_t sCountPerScan = 0;

/*
 * sets also sLastServoAngleInDegree to enable optimized servo movement and delays
 */
void ServoWrite(Servo aServo, int aValueDegree) {
    sLastServoAngleInDegree = aValueDegree;
    if (myCar.is2WDCar) {
        // My servo on the 2WD car is top down and therefore inverted
        aValueDegree = 180 - aValueDegree;
    }
    aServo.write(aValueDegree);
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
        } else if (tDistance < MIN_ALLOWED_DISTANCE_TO_SIDE_PRO) {
            tColor = COLOR_RED;
        }

        /*
         * Draw line
         */
        BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tDistance, tActualDegree, tColor, 3);
        tActualDegree += DEGREE_PER_STEP;
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
bool fillForwardDistancesInfoPro(ForwardDistancesInfoStruct* aForwardDistancesInfo, Servo aServoForUS, bool aShowValues,
bool aDoFirstValue) {

    Color_t tColor;

    // Values for forward scanning
    uint8_t tActualDegree = 0;
    uint8_t tDegreeIncrement = DEGREE_PER_STEP;
    int8_t tIndex = 0;
    int8_t tIndexDelta = 1;
    if (sLastServoAngleInDegree == 180) {
        // values for backward scanning
        tActualDegree = 180;
        tDegreeIncrement = -(DEGREE_PER_STEP);
        tIndex = MAX_DISTANCES_INDEX;
        tIndexDelta = -1;
    }
    if (!aDoFirstValue) {
        // skip first value, since it is equal to last value of last measurement
        tIndex += tIndexDelta;
        tActualDegree += tDegreeIncrement;
    }
    // Datasheet says: SG90 Micro Servo needs 100 millis per 60 degree angle => 300 ms per 180
    // I measured: SG90 Micro Servo needs 450 per 180 degree and per 2*90 degree, but 550 millis per 6*30 degree
    long tWaitDelayforServo = (abs(sLastServoAngleInDegree-tActualDegree) * 170) / 64;

    while (tIndex >= 0 && tIndex < NUMBER_OF_DISTANCES) {
        /*
         * rotate servo, wait and get distance
         */
        ServoWrite(aServoForUS, tActualDegree);

        delay(tWaitDelayforServo);
        unsigned int tDistance = getUSDistanceAsCentiMeterWithCentimeterTimeout(US_TIMEOUT_CENTIMETER_PRO);

        if ((tIndex == INDEX_FORWARD_1 || tIndex == INDEX_FORWARD_2) && tDistance < MIN_ALLOWED_DISTANCE_TO_FRONT_PRO) {
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
            if (tDistance == US_TIMEOUT_CENTIMETER_PRO) {
                tColor = COLOR_GREEN;
            } else if (tDistance < MIN_ALLOWED_DISTANCE_TO_SIDE_PRO) {
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
        tWaitDelayforServo = MILLIS_FOR_SERVO_30_DEGREES;
    }
    return true;
}

/*
 * Find min and max value prefer the headmost value if we have more than one choice
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
            tActualIndex = MAX_DISTANCES_INDEX - i;
            tDistance = aForwardDistancesInfo->ProcessedDistancesArray[tActualIndex];
        }
    }
}

/*
 * Assume the value of 20 and 40 degree are distances to a wall.
 * Return the clipped distance to the wall of the vector at 0 degree.
 * By changing MAX_DISTANCES_INDEX it can easily adopted to other degree values.
 */
uint8_t computeNeigbourValue(uint8_t a20DegreeValue, uint8_t a40DegreeValue, uint8_t aClipValue) {
    // assume actual = 40 Degree
    float tY40Degree = sin((PI / MAX_DISTANCES_INDEX) * 2) * a40DegreeValue;  // 40 Degree
    float tY20Degree = sin(PI / MAX_DISTANCES_INDEX) * a20DegreeValue; // 20 Degree

//    char tStringBuffer[] = "A=_______ L=_______";
//    dtostrf(tY40Degree, 7, 2, &tStringBuffer[2]);
//    tStringBuffer[9] = ' ';
//    dtostrf(tY20Degree, 7, 2, &tStringBuffer[12]);
//    BlueDisplay1.debugMessage(tStringBuffer);

    uint8_t tZeroDegree = aClipValue;

    /*
     * if tY40Degree == tY20Degree the tGradient is infinite (distance at 0 is infinite)
     */
    if (tY40Degree > tY20Degree) {
        float tX40Degree = cos((PI / MAX_DISTANCES_INDEX) * 2) * a40DegreeValue; // 40 Degree
        float tX20Degree = cos(PI / MAX_DISTANCES_INDEX) * a20DegreeValue; // 20 Degree

//        dtostrf(tX40Degree, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tX20Degree, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

        /*
         * process only if x value of 20 is be bigger than x values of 40, otherwise we are departing from the wall.
         * This implies, that the value of 0 measured by ultrasonic is expected to be valid!
         */
        if (tX20Degree > tX40Degree) {
            float tGradient = (tX40Degree - tX20Degree) / (tY40Degree - tY20Degree);
            float tXZeroDegree = tX20Degree - (tGradient * tY20Degree);

//        tStringBuffer[0] = 'G';
//        tStringBuffer[10] = 'B';
//        dtostrf(tGradient, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tXZeroDegree, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

            if (tXZeroDegree < 255) {
                tZeroDegree = tXZeroDegree + 0.5;
                if (tZeroDegree > aClipValue) {
                    tZeroDegree = aClipValue;
                }
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
void doWallDetectionPro(ForwardDistancesInfoStruct* aForwardDistancesInfo) {
    /*
     * First copy all raw values
     */
    memcpy(aForwardDistancesInfo->ProcessedDistancesArray, aForwardDistancesInfo->RawDistancesArray, NUMBER_OF_DISTANCES);
    uint8_t tLastValue = aForwardDistancesInfo->ProcessedDistancesArray[0];
    uint8_t tActualValue = aForwardDistancesInfo->ProcessedDistancesArray[1];
    uint8_t tBeforeValue = 0;
    uint8_t tLastBeforeValueIndex = 0xff;
    uint8_t tNextValue;
    uint8_t tActualDegree = 2 * DEGREE_PER_STEP;

    // i is index of ActualValue
    for (uint8_t i = 1; i < MAX_DISTANCES_INDEX; ++i) {
        tNextValue = aForwardDistancesInfo->ProcessedDistancesArray[i + 1];
        /*
         * check values at i and i-1
         */
        if (tLastValue < DISTANCE_FOR_WALL_DETECT && tActualValue < DISTANCE_FOR_WALL_DETECT) {
            /*
             * Wall detected -> adjust adjacent values
             */
            /*
             * Adjust next value
             */
            uint8_t tNextValueComputed = computeNeigbourValue(tActualValue, tLastValue, US_TIMEOUT_CENTIMETER_PRO);
            if (tNextValue > tNextValueComputed + 5) {

                //Adjust and draw next value if original value is greater
                aForwardDistancesInfo->ProcessedDistancesArray[i + 1] = tNextValueComputed;
                BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextValueComputed, tActualDegree,
                COLOR_BLACK, 1);
            }

            if (i >= 2) {
                /*
                 * adjust before value (i-2)
                 */
                uint8_t tBeforeValueComputed = computeNeigbourValue(tLastValue, tActualValue, US_TIMEOUT_CENTIMETER_PRO);
                if (tBeforeValue > tBeforeValueComputed + 5) {
                    //Adjust and draw next value if original value is greater
                    aForwardDistancesInfo->ProcessedDistancesArray[i - 2] = tBeforeValueComputed;
                    BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tBeforeValueComputed,
                            tActualDegree - 3 * DEGREE_PER_STEP, COLOR_BLACK, 1);
                    if (i >= 3 && (i - 3) != tLastBeforeValueIndex) {
                        /*
                         * adjust before-before value (i-3) if not already done
                         */
                        tBeforeValueComputed = computeNeigbourValue(aForwardDistancesInfo->ProcessedDistancesArray[i - 2],
                                tLastValue, US_TIMEOUT_CENTIMETER_PRO);
                        if (tBeforeValue > tBeforeValueComputed + 5) {
                            //Adjust and draw next value if original value is greater
                            aForwardDistancesInfo->ProcessedDistancesArray[i - 3] = tBeforeValueComputed;
                            BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tBeforeValueComputed,
                                    tActualDegree - 4 * DEGREE_PER_STEP, COLOR_BLACK, 1);
                        }
                        tLastBeforeValueIndex = i - 2;
                    }
                }
            }
        }
        tBeforeValue = tLastValue;
        tLastValue = tActualValue;
        tActualValue = tNextValue;
        tActualDegree += DEGREE_PER_STEP;
    }
    doPostProcess(aForwardDistancesInfo);
}

/*
 * Checks distances and returns degree to turn
 * 0 -> no turn, >0 -> turn left, <0 -> turn right
 */
int doCollisionDetectionPro(ForwardDistancesInfoStruct* aForwardDistancesInfo) {
    int tDegreeToTurn = 0;
    /*
     * First check if free ahead
     */
    if (aForwardDistancesInfo->ProcessedDistancesArray[INDEX_FORWARD_1] > MIN_ALLOWED_DISTANCE_TO_FRONT_PRO
            && aForwardDistancesInfo->ProcessedDistancesArray[INDEX_FORWARD_2] > MIN_ALLOWED_DISTANCE_TO_FRONT_PRO) {
        /*
         * Go Forward, but check if our side is near to the wall and make corrections
         */
        uint8_t tIndexOfMinDistance = aForwardDistancesInfo->IndexOfMinDistance;
        if ((tIndexOfMinDistance <= 3 || tIndexOfMinDistance >= 6)
                && aForwardDistancesInfo->MinDistance < MIN_ALLOWED_DISTANCE_TO_SIDE_PRO) {
//            BlueDisplay1.debugMessage("min=");
//            BlueDisplay1.debug(aForwardDistancesInfo->IndexOfMinDistance);
//            BlueDisplay1.debug(aForwardDistancesInfo->MinDistance);
            if (tIndexOfMinDistance <= 3) {
                /*
                 * Found wall at right => turn left
                 */
                tDegreeToTurn = tIndexOfMinDistance * DEGREE_PER_STEP;
            } else {
                /*
                 * Found wall at left => turn right
                 */
                tDegreeToTurn = -((MAX_DISTANCES_INDEX - tIndexOfMinDistance) * DEGREE_PER_STEP);
            }
        }
    } else {
//        BlueDisplay1.debugMessage("max=");
//        BlueDisplay1.debug(aForwardDistancesInfo->IndexOfMaxDistance);
//        BlueDisplay1.debug(aForwardDistancesInfo->MaxDistance);

        /*
         * Not free ahead, must turn, check if another forward direction is suitable
         */
        if (aForwardDistancesInfo->MaxDistance > MIN_ALLOWED_DISTANCE_TO_FRONT_PRO) {
            /*
             * Just turn
             */
            tDegreeToTurn = aForwardDistancesInfo->IndexOfMaxDistance * DEGREE_PER_STEP - 90;
        } else {
            /*
             * Must go back => Turn by 180 degree
             */
            tDegreeToTurn = 180;
        }
    }
    return tDegreeToTurn;
}

void driveAutonomous(bool * aRunFlagPtr, bool (*afillForwardDistancesInfoFunction)(ForwardDistancesInfoStruct*, Servo, bool,
bool), int (*aCollisionDetectionFunction)(ForwardDistancesInfoStruct*)) {
    EncoderMotorControl::EnableValuesPrint = false;
    uint16_t tStartCount = 0;

    while (*aRunFlagPtr) {
        /*
         * Check if driving is enabled
         */
        if (sStepMode == MODE_CONTINUOUS || (sStepMode == MODE_STEP && (!myCar.isStopped() || sDoStep))
                || (sStepMode == MODE_SINGLE_STEP && sDoStep)) {
            bool tJustStarted = sDoStep;
            sDoStep = false;

            /*
             * Handle step modes
             */
            if (sStepMode == MODE_SINGLE_STEP) {
                // turn and go fixed distance
                myCar.rotateCar(sDegreeToTurn);
                myCar.goDistanceCentimeter(CENTIMETER_PER_RIDE_PRO, &loopGUI);
            } else if (myCar.isStopped()) {
                // last step finished -> add to path, rotate and start again
                addToPath(myCar.rightMotorControl.LastRideDistanceCount, sDegreeToTurn);
                myCar.rotateCar(sDegreeToTurn, TURN_BACKWARD);
                myCar.startAndWaitForFullSpeed();
                tJustStarted = true;
            }

            bool tShowInfos = (sActualPage == PAGE_AUTOMATIC_CONTROL);
            /*
             * Clear old decision marker
             */
            if (tShowInfos && (sDegreeToTurn % 30) != 0) {
                // clear old bar
                printCollisionDecision(sDegreeToTurn, CENTIMETER_PER_RIDE_PRO, true);
            }

            /*
             * compute distance driven for one scan
             */
            if (tStartCount == 0) {
                uint16_t tActualCount = myCar.leftMotorControl.DistanceCount;
                sCountPerScan = tStartCount - tActualCount;
                tStartCount = tActualCount;
            }

            /*
             * The magic happens HERE
             */
            bool tInfoWasProcessed = afillForwardDistancesInfoFunction(&ForwardDistancesInfo, sServoUS, tShowInfos, tJustStarted);
            doWallDetectionPro(&ForwardDistancesInfo);
            sDegreeToTurn = aCollisionDetectionFunction(&ForwardDistancesInfo);

            /*
             * Show distance info if not already done
             */
            if (!tInfoWasProcessed && tShowInfos) {
                drawForwardDistancesInfos(&ForwardDistancesInfo);
            }
            printCollisionDecision(sDegreeToTurn, CENTIMETER_PER_RIDE_PRO, false);

            /*
             * Decide if we have to stop or to just stay running running (and synchronize)
             */
            if (sDegreeToTurn != 0 || sStepMode == MODE_SINGLE_STEP) {
                myCar.stopCar();
                tStartCount = 0;
                /*
                 * Show last ride in path
                 */
                if (sStepMode == MODE_SINGLE_STEP) {
                    addToPath(CENTIMETER_PER_RIDE_PRO, sLastDegreeToTurn);
                } else {
                    addToPath(myCar.rightMotorControl.LastRideDistanceCount, sLastDegreeToTurn);
                }
                sLastDegreeToTurn = sDegreeToTurn;
            } else {
                /*
                 * overwrite last path element with actual riding distance and try to synchronize motors
                 */
                insertToPath(myCar.rightMotorControl.DistanceCount, sLastDegreeToTurn);
                myCar.rightMotorControl.syncronizeMotor(&myCar.leftMotorControl, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
            }
            if (sActualPage == PAGE_SHOW_PATH) {
                DrawPath();
            }
        }

        /*
         * check for user input
         */
        loopGUI();
    }
    /*
     * End of test2
     */
    EncoderMotorControl::EnableValuesPrint = true;
    ServoWrite(sServoUS, 90);
}

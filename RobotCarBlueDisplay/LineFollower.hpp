/*
 * LineFollower.hpp
 *
 *  Implements line follower driving for a 2 or 4 wheel car.
 *  A TCRT 5000 3-channel sensor is used. This sensor is very delicate to calibrate :-(.
 *  Features single step mode.
 *
 *  Copyright (C) 2026  Armin Joachimsmeyer
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _ROBOT_CAR_LINE_FOLLOWER_HPP
#define _ROBOT_CAR_LINE_FOLLOWER_HPP

// This block must be located after the includes of other *.hpp files
#define LOCAL_INFO  // This enables info output only for this file
#include "LocalDebugLevelStart.h"

/*
 * Enable LINE_FOLLOWER_SENSORS_ARE_MOUNTED_AT_BACK_OF_CAR, if the sensors are mounted at back of your car.
 * In this case, the directions must be switched, e.g. we must specify DIRECTION_BACKWARD to go forward.
 */
//#define LINE_FOLLOWER_SENSORS_ARE_MOUNTED_AT_BACK_OF_CAR
#if defined(LINE_FOLLOWER_SENSORS_ARE_MOUNTED_AT_BACK_OF_CAR)
// Here, the sensors are mounted at the back, so we must switch forward and backward directions
#define LINE_FOLLOWER_DIRECTION_FORWARD     DIRECTION_BACKWARD
#define LINE_FOLLOWER_DIRECTION_BACKWARD    DIRECTION_FORWARD
#define LINE_FOLLOWER_TURN_FORWARD          TURN_BACKWARD
#define LINE_FOLLOWER_TURN_BACKWARD         TURN_FORWARD
#else
#define LINE_FOLLOWER_DIRECTION_FORWARD     DIRECTION_FORWARD
#define LINE_FOLLOWER_DIRECTION_BACKWARD    DIRECTION_BACKWARD
#define LINE_FOLLOWER_TURN_FORWARD          TURN_FORWARD
#define LINE_FOLLOWER_TURN_BACKWARD         TURN_BACKWARD
#endif

uint8_t sOldSensorState; // Used to detect changes of sensor state
bool sForceMovementForNextStep = false; // if true => do not stop on all the sensors above the line which is used as a stop mark
unsigned long sMillisOfLastSensorChange = 0;

/***************************************************
 * Code for line follower mode
 ***************************************************/
// use INPUT_PULLUP, to avoid moving if sensor is not connected
void initLineFollowerSensors() {
    pinMode(LINE_FOLLOWER_LEFT_SENSOR_PIN, INPUT_PULLUP);
    pinMode(LINE_FOLLOWER_MID_SENSOR_PIN, INPUT_PULLUP);
    pinMode(LINE_FOLLOWER_RIGHT_SENSOR_PIN, INPUT_PULLUP);
}

void initLineFollowerSensorsAndPrint(Print *aSerial) {
    aSerial->println(F("Left sensor pin=" STR(LINE_FOLLOWER_LEFT_SENSOR_PIN)));
    aSerial->println(F("Middle sensor pin=" STR(LINE_FOLLOWER_MID_SENSOR_PIN)));
    aSerial->println(F("Right sensor pin=" STR(LINE_FOLLOWER_RIGHT_SENSOR_PIN)));
    initLineFollowerSensors(); // put after printing else formatter crashes
}

void startLineFollower() {
    sForceMovementForNextStep = true;
    sOldSensorState = 0;
}

void stopLineFollower() {
    sForceMovementForNextStep = false;
    startStopAutomomousDrive(false, MODE_LINE_FOLLOWER);
}

#define SENSOR_STATE_0_DURATION_FOR_STOP_BAR_MILLIS     80

/*
 * Get values from the TCRT 5000 3-channel sensor
 * input is high, if sensor sees dark, i.e. sees dark line
 */
uint8_t getLineSensorState() {
    uint8_t tLineSensorState = 0;
    if (digitalRead(LINE_FOLLOWER_LEFT_SENSOR_PIN) == LOW) tLineSensorState = 0b001;   // set bit 0 if input is LOW
    if (digitalRead(LINE_FOLLOWER_MID_SENSOR_PIN) == LOW) tLineSensorState |= 0b010;   // set bit 1 if input is LOW
    if (digitalRead(LINE_FOLLOWER_RIGHT_SENSOR_PIN) == LOW) tLineSensorState |= 0b100; // set bit 2 if input is LOW
#if defined(LINE_FOLLOWER_ENABLE_WHITE_LINE_PIN)
    if(!digitalRead(LINE_FOLLOWER_ENABLE_WHITE_LINE_PIN)) {
        // If LOW follow white line. This just inverts the reading of the sensors
        tLineSensorState = tLineSensorState ^ 0x07;
    }
#endif
    return tLineSensorState;
}

/*
 * Mode MODE_STEP_TO_NEXT_TURN is treated as MODE_CONTINUOUS for line follower
 */
void driveLineFollowerModeOneStep() {
    if (sDoStep) {
        uint8_t tNewSensorState = getLineSensorState();

        unsigned int tMillisSinceLastSensorChange = millis() - sMillisOfLastSensorChange; // reduced resolution here
        if (sOldSensorState != tNewSensorState || sForceMovementForNextStep) {
            sMillisOfLastSensorChange = millis();
        } else if (sOldSensorState == 0 && tMillisSinceLastSensorChange >= SENSOR_STATE_0_DURATION_FOR_STOP_BAR_MILLIS) {
            /*
             * We are passing a stop mark / black bar
             */
            sOldSensorState = 8; // set dummy sensor state to force stopping now
        }

        /*
         * Dark means: Sensor sees line.
         * According to the 8 different states of the 3 sensor inputs, we perform the following actions:
         * 0 0b000 - All sensors are dark or not connected / over stop mark -> stop or go forward after sharp turn or start of move
         * 1 0b001 - Mid and right sensors are dark -> sharp right
         * 2 0b010 - Left and right sensors are dark -> panic stop, because this is unexpected
         * 3 0b011 - Only right sensor is dark -> right
         * 4 0b100 - Mid and left sensors are dark -> sharp left
         * 5 0b101 - Only mid sensor is dark -> forward
         * 6 0b110 - Only left sensor is dark -> left
         * 7 0b111 - All sensors are not dark -> stop or go backward after turn
         */

        /*
         * If sensor input does change or we have a start of a single step, we need to start or change movement!
         */
        if ((sOldSensorState != tNewSensorState) || sForceMovementForNextStep) {
            if (sStepMode == MODE_SINGLE_STEP || sForceMovementForNextStep) {
                // Here we have MODE_SINGLE_STEP but no start
                if (!sForceMovementForNextStep) {
                    // Here sensor state has changed in single step mode
                    if (!RobotCar.isStopped()) {
                        RobotCar.stop(STOP_MODE_BRAKE);
                    }
                    sDoStep = false;
                    sForceMovementForNextStep = true; // Enable next single step if sDoStep is true.
                    return;
                }
            }

#if defined(LOCAL_INFO)
            // This info can be seen on the BD screen and in the BD log
            Serial.print(F("SensorState "));
            Serial.print(sOldSensorState, BIN);
            Serial.print(F(" -> "));
            Serial.print(tNewSensorState, BIN);
            if (sStepMode != MODE_SINGLE_STEP) {
                Serial.print(F(" = "));
                Serial.print(tMillisSinceLastSensorChange);
                Serial.print(F(" ms"));
            }
            Serial.print(F(": "));
#endif

            switch (tNewSensorState) {
            case 0b000:
                // All sensors are dark or not connected -> stop or go forward after sharp turn
                if (sOldSensorState == 0b001 || sOldSensorState == 0b100 || sForceMovementForNextStep) { // test for sharp turns or movement just started
                    DEBUG_PRINTLN(F("Forward during sharp turn or start"));
                    RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, LINE_FOLLOWER_DIRECTION_FORWARD);
                } else {
                    INFO_PRINTLN(F("Stop at bar")); // we are on a T junction / stop mark now. I measured 130 ms to 160 mm over the 25 mm bar.
                    stopLineFollower();
                }
                break;
            case 0b001:
                // Mid and right sensors are dark -> sharp right
                DEBUG_PRINTLN(F("Turn sharp right"));
                RobotCar.startRotate(-180, LINE_FOLLOWER_TURN_FORWARD); // here both LINE_FOLLOWER_TURN_FORWARD and TURN_IN_PLACE work
                break;
            case 0b010:
                // Left and right sensors are dark -> panic stop, because this is unexpected
                INFO_PRINTLN(F("Panic stop"));
                stopLineFollower();
                break;
            case 0b011:
                // Only right sensor is dark -> right
                DEBUG_PRINTLN(F("Turn right"));
                RobotCar.startRotate(-180, LINE_FOLLOWER_TURN_FORWARD);
                break;
            case 0b100:
                // Mid and left sensors are dark -> sharp left
                DEBUG_PRINTLN(F("Turn sharp left"));
                RobotCar.startRotate(180, LINE_FOLLOWER_TURN_FORWARD); // here both LINE_FOLLOWER_TURN_FORWARD and TURN_IN_PLACE work
                break;
            case 0b101:
                // Only mid sensor is dark -> forward
                DEBUG_PRINTLN(F("Forward"));
                RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, LINE_FOLLOWER_DIRECTION_FORWARD);
                break;
            case 0b110:
                // Only left sensor is dark -> left
                DEBUG_PRINTLN(F("Turn left"));
                RobotCar.startRotate(180, LINE_FOLLOWER_TURN_FORWARD);
                break;
            case 0b111:
                // All sensors are not dark
                // We are after the end of line or we left area while turning -> stop or go backward after turn
                if (sOldSensorState == 0b110 || sOldSensorState == 0b011) {
                    DEBUG_PRINTLN(F("Backward after turn"));
                    RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, LINE_FOLLOWER_DIRECTION_BACKWARD);
                } else {
                    INFO_PRINTLN(F("Stop no line detected"));
                    stopLineFollower();
                }
                break;
            }

            sForceMovementForNextStep = false;
            if (sStepMode == MODE_SINGLE_STEP) {
                delayMillisHandleEventsAndBreakIfEventJustReceived(50); // Otherwise we may have spikes
            }
        }
        sOldSensorState = tNewSensorState;
    }
}
#include "LocalDebugLevelEnd.h"

#endif // _ROBOT_CAR_LINE_FOLLOWER_HPP

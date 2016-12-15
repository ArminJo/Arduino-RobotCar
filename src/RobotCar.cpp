/*
 *  RobotCar.cpp
 *  Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
 *  To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the area.
 *  Manual control is by a GUI implemented with a Bluetooth HC-05 Module and the BlueDisplay library.
 *
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

#include <Arduino.h>
#include <AutonomousDrive.h>

#include "RobotCar.h"
#include "RobotCarGui.h"
#include "ArminsUtils.h"

#include "EncoderMotorControl.h"

// Pin 13 has an LED connected on most Arduino boards.
const int LED_PIN = 13;

// if connected to ground we have a 2 WD CAR
const int TWOWD_DETECTION_PIN = 12;

// Must use pin 3 and 4 on Arduino, since they are connected to INT0 and INT1
const int DISTANCE_SENSOR_LEFT_PIN = 3;
const int DISTANCE_SENSOR_RIGHT_PIN = 4;

/*
 * Car Control
 */
CarControl myCar;

/*
 * Servo + US stuff
 */
// must not be constant, since then we get an undefined reference error at link time
uint8_t ECHO_IN_PIN = A1;
uint8_t TRIGGER_OUT_PIN = A0;

Servo sServoUS;

#define CENTIMETER_PER_RIDE 20

#define MINIMUM_DISTANCE_TO_SIDE 21
#define MINIMUM_DISTANCE_TO_FRONT 35

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_PIN, OUTPUT);

    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    pinMode(DISTANCE_SENSOR_RIGHT_PIN, INPUT);
    pinMode(DISTANCE_SENSOR_LEFT_PIN, INPUT);

    pinMode(TRIGGER_OUT_PIN, OUTPUT);
    pinMode(ECHO_IN_PIN, INPUT);

    EncoderMotorControl::enableInterruptOnRisingSlope(INT0);
    EncoderMotorControl::enableInterruptOnRisingSlope(INT1);
    EncoderMotorControl::EnableValuesPrint = true;

    sServoUS.attach(10);
    sLastServoAngleInDegree = 90;
    ServoWrite(sServoUS, sLastServoAngleInDegree);

// initialize motors
    myCar.init(TWOWD_DETECTION_PIN);
// reset all values
    resetGUIControls();
    resetPathData();

    setupGUI();
}

void loop() {

    loopGUI();

    if (sActualPage == PAGE_MANUAL_CONTROL) {
        if (sRunManual) {
            /*
             * Direct control by GUI
             */
            myCar.updateMotors();
            myCar.rightMotorControl.syncronizeMotor(&myCar.leftMotorControl, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
        }
        checkAndShowDistancePeriodically(sGetDistancePeriod);
    }
    /*
     * Test loop 1
     */
    if (sRunTestSimple) {
        driveAutonomous(&sRunTestSimple, &fillForwardDistancesInfoSimple, &doCollisionDetectionSimple);
    }
    /*
     * Test loop 2
     */
    if (sRunTestPro) {
        driveAutonomous(&sRunTestPro, &fillForwardDistancesInfoPro, &doCollisionDetectionPro);
    }
}

/*
 * Get 9 distances starting at 0 degree (right) increasing by 20 degree up to 180 degree (left)
 */
bool fillForwardDistancesInfoSimple(ForwardDistancesInfoStruct* aForwardDistancesInfo, Servo aServoForUS, bool aShowValues,
bool aDoFirstValue) {
    int tDegree = 0;
    aForwardDistancesInfo->MaxDistance = 0;
    aForwardDistancesInfo->MinDistance = 255;
    aForwardDistancesInfo->IndexOfMaxDistance = 0;
    aForwardDistancesInfo->IndexOfMinDistance = 0;
    int tDelay = 600;
    for (int i = 0; i < 7; ++i) {
        sServoUS.write(tDegree);
        delay(tDelay);
        tDelay = 120;
        unsigned int tActualDistance = getUSDistanceAsCentiMeter();
        /*
         * Get 9 Distances & Check for Max and Min distances
         * The 9 Distances are Stored in aForwardDistancesInfo->RawDistancesArray
         */
        aForwardDistancesInfo->RawDistancesArray[i] = tActualDistance;
        tDegree += DEGREE_PER_STEP;
        if (tActualDistance > aForwardDistancesInfo->MaxDistance) {
            aForwardDistancesInfo->MaxDistance = tActualDistance;
            aForwardDistancesInfo->IndexOfMaxDistance = i;
        }
        if (tActualDistance < aForwardDistancesInfo->MinDistance) {
            aForwardDistancesInfo->MinDistance = tActualDistance;
            aForwardDistancesInfo->IndexOfMinDistance = i;
        }
    }
    return false;
}

/*
 * Checks distances and returns degree to turn
 * 0 -> no turn, >0 -> turn left, <0 -> turn right
 */
int doCollisionDetectionSimple(ForwardDistancesInfoStruct* aForwardDistancesInfo) {
    // if left three distances are all less than 21 centimeter then turn right.
    if (aForwardDistancesInfo->ProcessedDistancesArray[INDEX_LEFT] <= MINIMUM_DISTANCE_TO_SIDE
            && ForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT - 1] <= MINIMUM_DISTANCE_TO_SIDE
            && ForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT - 2] <= MINIMUM_DISTANCE_TO_SIDE) {
        return -90;
        // check right three distances are all less then 21 centimeter than turn left.
    } else if (ForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT] <= MINIMUM_DISTANCE_TO_SIDE
            && ForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT + 1] <= MINIMUM_DISTANCE_TO_SIDE
            && ForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT + 2] <= MINIMUM_DISTANCE_TO_SIDE) {
        return 90;
        // check front distance is longer then 35 centimeter than do not turn.
    } else if (ForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_1] >= MINIMUM_DISTANCE_TO_FRONT
            && ForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_2] >= MINIMUM_DISTANCE_TO_FRONT) {
        return 0;
    } else if (ForwardDistancesInfo.MaxDistance >= MINIMUM_DISTANCE_TO_SIDE) {
        /*
         * here front distance is less then 35 centimeter:
         * go to max side distance
         */
        // formula to convert index to degree.
        return -90 + DEGREE_PER_STEP * ForwardDistancesInfo.IndexOfMaxDistance;
    } else {
        // Turn backwards.
        return 180;
    }
}

// ISR for PIN PD2 / RIGHT
ISR(INT0_vect) {
    myCar.rightMotorControl.handleEncoderInterrupt();
}

// ISR for PIN PD3 / LEFT
ISR(INT1_vect) {
    myCar.leftMotorControl.handleEncoderInterrupt();
}


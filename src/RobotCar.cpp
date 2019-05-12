/*
 *  RobotCar.cpp
 *  Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
 *  To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the area.
 *  Manual control is by a GUI implemented with a Bluetooth HC-05 Module and the BlueDisplay library.
 *  Just overwrite the 2 functions myOwnFillForwardDistancesInfo() and myOwnDoCollisionDetection() to test your own skill.
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

#include <digitalWriteFast.h>
#include <LightweightServo.h>
#include <HCSR04.h>

#include <EncoderMotorControl.h>

#include "AutonomousDrive.h"

#include "RobotCar.h"
#include "RobotCarGui.h"

/*
 * Car Control
 */
CarControl myCar;

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    pinMode(RIGHT_ENCODER_PIN, INPUT);
    pinMode(LEFT_ENCODER_PIN, INPUT);

    pinMode(TRIGGER_OUT_PIN, OUTPUT);
    pinMode(DEBUG_OUT_PIN, OUTPUT);
    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);

    EncoderMotorControl::enableInterruptOnBothEdges(INT0);
    EncoderMotorControl::enableInterruptOnBothEdges(INT1);
    EncoderMotorControl::EnableValuesPrint = true;

#ifdef DISABLE_SERVO_TIMER_AUTO_INITIALIZE
    initLightweightServoPin9And10();
#endif
    // set Laser Servo
    write10(90);
    // set ultrasonic servo
    US_ServoWrite(90);

// initialize motors
    myCar.init(TWO_WD_DETECTION_PIN);
// reset all values
    resetGUIControls();
    resetPathData();

    setupGUI();

}

void loop() {
    digitalToggleFast(LED_BUILTIN);
    if (!BlueDisplay1.isConnectionEstablished()) {
        if (millis() > 15000) {
            /*
             * Start automatically if no Bluetooth connection
             */
            doStartStopAutomomousDrive(NULL, true);
        }
    }

    /*
     * check for user input
     */
    loopGUI();

    if (sActualPage == PAGE_MANUAL_CONTROL && sStarted) {
        /*
         * Direct control by GUI
         */
        myCar.updateMotors();
        myCar.rightMotorControl.synchronizeMotor(&myCar.leftMotorControl, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
    }

    /*
     * Test loops
     */
    if (sRunOwnTest || sRunAutonomousDrive) {
        bool (*tfillForwardDistancesInfoFunction)(ForwardDistancesInfoStruct*, bool, bool);
        int (*tCollisionDetectionFunction)(ForwardDistancesInfoStruct*);
        if (sRunOwnTest) {
            tfillForwardDistancesInfoFunction = &myOwnFillForwardDistancesInfo;
            tCollisionDetectionFunction = &myOwnDoCollisionDetection;
        } else {
            tfillForwardDistancesInfoFunction = &fillForwardDistancesInfo;
            tCollisionDetectionFunction = &doCollisionDetectionPro;
        }
        EncoderMotorControl::EnableValuesPrint = false;
        while (sRunAutonomousDrive || sRunOwnTest) {
            driveAutonomous(tfillForwardDistancesInfoFunction, tCollisionDetectionFunction);
            /*
             * check for user input.
             */
            loopGUI();
        }
        /*
         * End of test loops. myCar.isStopped() is true here
         */
        if (sStepMode != MODE_SINGLE_STEP) {
            // add last driven distance to path
            insertToPath(myCar.rightMotorControl.LastRideDistanceCount, sLastDegreesTurned, true);
        }

        EncoderMotorControl::EnableValuesPrint = true;
        US_ServoWrite(90);
    }
}

/*
 * Get 9 distances starting at 0 degree (right) increasing by 20 degree up to 180 degree (left)
 */
bool myOwnFillForwardDistancesInfo(ForwardDistancesInfoStruct* aForwardDistancesInfo, bool aShowValues,
bool aDoFirstValue) {
    int tDegree = 180;
    int tDelay = 600;
    for (int i = 0; i < NUMBER_OF_DISTANCES; ++i) {
        // My servo is top down and therefore inverted
        tDegree = 180 - tDegree;
        write10(tDegree);
        delay(tDelay);
        tDelay = 100;
        unsigned int tActualDistance = getUSDistanceAsCentiMeter();
        /*
         * Get 9 Distances & Check for Max and Min distances
         * The 9 Distances are Stored in aForwardDistancesInfo->RawDistancesArray
         */
        aForwardDistancesInfo->RawDistancesArray[i] = tActualDistance;
        tDegree -= DEGREES_PER_STEP;
    }
    return false;
}

/*
 * Checks distances and returns degree to turn
 * 0 -> no turn, >0 -> turn left, <0 -> turn right
 */
int myOwnDoCollisionDetection(ForwardDistancesInfoStruct* aForwardDistancesInfo) {
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
        return -90 + DEGREES_PER_STEP * ForwardDistancesInfo.IndexOfMaxDistance;
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


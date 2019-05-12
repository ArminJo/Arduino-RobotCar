/*
 * CarControl.cpp
 *
 *  Contains functions for control of the 2 motors of a car like setDirection, goDistanceCentimeter() and rotateCar().
 *  Checks input of PIN aPinFor2WDDetection since we need different factors for rotating a 4 wheel and a 2 wheel car.
 *
 *  Needs EncoderMotorControl.cpp
 *
 *  Created on: 16.09.2016
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include "RobotCarGui.h"
#include "CarControl.h"
#include "EncoderMotorControl.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

CarControl::CarControl() {
}

//CarControl::~CarControl() {
//}

void CarControl::init(uint8_t aPinFor2WDDetection) {
    pinMode(aPinFor2WDDetection, INPUT_PULLUP);

    // Select which 'port' M1, M2, M3 or M4. In this case, M1
    Adafruit_DCMotor *sLeftMotor = AFMS.getMotor(1);
    leftMotorControl.init(sLeftMotor);

    // You can also make another motor on port M2
    Adafruit_DCMotor *sRightMotor = AFMS.getMotor(2);
    rightMotorControl.init(sRightMotor);
    AFMS.begin();  // create with the default frequency 1.6KHz
    is2WDCar = !digitalRead(aPinFor2WDDetection);
    isDirectionForward = true;
}

void CarControl::setSpeedCompensated(uint8_t aSpeed) {
    rightMotorControl.setSpeedCompensated(aSpeed);
    leftMotorControl.setSpeedCompensated(aSpeed);
}

void CarControl::activateMotors() {
    rightMotorControl.activate();
    leftMotorControl.activate();
}

void CarControl::shutdownMotors(bool doBrake) {
    rightMotorControl.shutdownMotor(doBrake);
    leftMotorControl.shutdownMotor(doBrake);
}

void CarControl::updateMotors() {
    rightMotorControl.updateMotor();
    leftMotorControl.updateMotor();
}

void CarControl::resetAndShutdownMotors() {
    rightMotorControl.resetAndShutdown();
    leftMotorControl.resetAndShutdown();
    isDirectionForward = true;
}

/*
 * This stops motors
 */
void CarControl::setDirection(bool goForward) {
    isDirectionForward = goForward;
    rightMotorControl.setDirection(goForward);
    leftMotorControl.setDirection(goForward);
}

/*
 * initialize motorInfo fields DirectionForward, ActualMaxSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void CarControl::initGoDistanceCentimeter(int aDistanceCentimeter) {
    rightMotorControl.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT);
    leftMotorControl.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT);
}

/**
 * Wait until distance is reached
 * @param  aLoopCallback called until car has stopped to avoid blocking
 *
 */
void CarControl::goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void)) {
    initGoDistanceCentimeter(aDistanceCentimeter);
    waitUntilCarStopped(aLoopCallback);
}

/*
 * Start motor for "infinite" distance and then blocking wait until both motors are at full speed
 */
void CarControl::startAndWaitForFullSpeed() {
    initGoDistanceCentimeter(3200);
    /*
     * blocking wait for start
     */
    do {
        rightMotorControl.updateMotor();
        leftMotorControl.updateMotor();
    } while (rightMotorControl.State != MOTOR_STATE_FULL_SPEED || leftMotorControl.State != MOTOR_STATE_FULL_SPEED);
}

/*
 * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_FULL_SPEED to MOTOR_STATE_RAMP_DOWN
 * Use DistanceCountAfterRampUp as ramp down count
 * Blocking wait for stop
 */
void CarControl::stopCar() {
    if (isStopped()) {
        return;
    }
    rightMotorControl.NextChangeMaxTargetCount = rightMotorControl.DistanceCount;
    rightMotorControl.TargetDistanceCount = rightMotorControl.DistanceCount + rightMotorControl.DistanceCountAfterRampUp;
    leftMotorControl.NextChangeMaxTargetCount = leftMotorControl.DistanceCount;
    leftMotorControl.TargetDistanceCount = leftMotorControl.DistanceCount + leftMotorControl.DistanceCountAfterRampUp;

    /*
     * blocking wait for stop
     */
    do {
        rightMotorControl.updateMotor();
        leftMotorControl.updateMotor();
    } while (!isStopped());
}

/*
 * Non blocking version of wait
 */
void CarControl::waitUntilCarStopped(void (*aLoopCallback)(void)) {
    do {
        rightMotorControl.updateMotor();
        leftMotorControl.updateMotor();
        if (aLoopCallback != NULL) {
            aLoopCallback();
        }
    } while (!isStopped());
}

/**
 * Blocking version of wait
 */
void CarControl::waitUntilCarStopped() {
    do {
        rightMotorControl.updateMotor();
        leftMotorControl.updateMotor();
    } while (!isStopped());
}

bool CarControl::isState(uint8_t aState) {
    return (rightMotorControl.State == aState && leftMotorControl.State == aState);
}

bool CarControl::isStopped() {
    return isState(MOTOR_STATE_STOPPED);
}

bool CarControl::isStarted() {
    return isState(MOTOR_STATE_FULL_SPEED);
}

/**
 * @param  if aUseSlowSpeed is true then use slower speed (1.5 times MinSpeed) for rotation to be more exact
 */
void CarControl::initRotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed) {
    int tDistanceCountRight;
    int tDistanceCountLeft;
    float tFactor;
    if (is2WDCar) {
        tFactor = FACTOR_DEGREE_TO_COUNT_2WD_CAR;
    } else {
        tFactor = FACTOR_DEGREE_TO_COUNT_4WD_CAR;
    }
    if (aRotationDegrees > 0) {
// turn left
        tDistanceCountRight = (aRotationDegrees * tFactor) + 0.5;
        tDistanceCountLeft = 0;
        if (aTurnDirection == TURN_IN_PLACE) {
            tDistanceCountLeft = -tDistanceCountRight / 2;
            tDistanceCountRight = tDistanceCountRight - tDistanceCountRight / 2;
        }
    } else {
// turn right
        tDistanceCountLeft = (-aRotationDegrees * tFactor) + 0.5;
        tDistanceCountRight = 0;
        if (aTurnDirection == TURN_IN_PLACE) {
            tDistanceCountRight = -tDistanceCountLeft / 2;
            tDistanceCountLeft = tDistanceCountLeft - tDistanceCountLeft / 2;
        }
    }
    if (aTurnDirection == TURN_BACKWARD) {
        int tDistanceCountTemp = tDistanceCountRight;
        tDistanceCountRight = -tDistanceCountLeft;
        tDistanceCountLeft = -tDistanceCountTemp;

    }
// This in turn sets ActualMaxSpeed to MaxSpeed.
    rightMotorControl.initGoDistanceCount(tDistanceCountRight);
    leftMotorControl.initGoDistanceCount(tDistanceCountLeft);
    if (aUseSlowSpeed) {
// adjust MaxSpeed at last
        rightMotorControl.ActualMaxSpeed = rightMotorControl.MinSpeed + rightMotorControl.MinSpeed / 2;
        leftMotorControl.ActualMaxSpeed = leftMotorControl.MinSpeed + leftMotorControl.MinSpeed / 2;
    }
}

/**
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 */
void CarControl::rotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed) {
    if (aRotationDegrees != 0) {
        initRotateCar(aRotationDegrees, aTurnDirection, aUseSlowSpeed);
        waitUntilCarStopped();
    }
}

/**
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 * @param  aLoopCallback avoid blocking and call aLoopCallback on waiting for stop
 */
void CarControl::rotateCar(int16_t aRotationDegrees, void (*aLoopCallback)(void), uint8_t aTurnDirection, bool aUseSlowSpeed) {
    if (aRotationDegrees != 0) {
        initRotateCar(aRotationDegrees, aTurnDirection, aUseSlowSpeed);
        waitUntilCarStopped(aLoopCallback);
    }
}

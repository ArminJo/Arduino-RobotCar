/*
 * CarMotorControl.cpp
 *
 *  Contains functions for control of the 2 motors of a car like setDirection, goDistanceCentimeter() and rotateCar().
 *  Checks input of PIN aPinFor2WDDetection since we need different factors for rotating a 4 wheel and a 2 wheel car.
 *
 *  Needs EncoderMotor.cpp
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

#include <Arduino.h>
#include <CarMotorControl.h>
#include <EncoderMotor.h>

EncoderMotor rightEncoderMotor;
EncoderMotor leftEncoderMotor;

CarMotorControl::CarMotorControl() { // @suppress("Class members should be properly initialized")
}

//CarMotorControl::~CarMotorControl() {
//}

void CarMotorControl::init(uint8_t aPinFor2WDDetection) {
    pinMode(aPinFor2WDDetection, INPUT_PULLUP);

    leftEncoderMotor.init(0);
    rightEncoderMotor.init(1);
    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    EncoderMotor::enableINT0AndINT1Interrupts();

    is2WDCar = !digitalRead(aPinFor2WDDetection);
    isDirectionForward = true;
}

void CarMotorControl::setSpeedCompensated(uint8_t aSpeed) {
    rightEncoderMotor.setSpeedCompensated(aSpeed);
    leftEncoderMotor.setSpeedCompensated(aSpeed);
}

void CarMotorControl::activateMotors() {
    rightEncoderMotor.activate();
    leftEncoderMotor.activate();
}

/*
 * Stop car
 */
void CarMotorControl::shutdownMotors(bool doBrake) {
    rightEncoderMotor.shutdownMotor(doBrake);
    leftEncoderMotor.shutdownMotor(doBrake);
}

/*
 * If motor is accelerating or decelerating then updateMotor needs to be called at a fast rate otherwise it will not work correctly
 */
bool CarMotorControl::needsFastUpdates() {
    return (rightEncoderMotor.State == MOTOR_STATE_RAMP_DOWN || rightEncoderMotor.State == MOTOR_STATE_RAMP_UP
            || leftEncoderMotor.State == MOTOR_STATE_RAMP_DOWN || leftEncoderMotor.State == MOTOR_STATE_RAMP_UP);
}

void CarMotorControl::updateMotors() {
    rightEncoderMotor.updateMotor();
    leftEncoderMotor.updateMotor();
}

/*
 * Stop car and reset all control values as speed, distances, debug values to 0x00
 * Leave calibration and compensation values unaffected.
 */
void CarMotorControl::resetAndShutdownMotors() {
    rightEncoderMotor.resetAndShutdown();
    leftEncoderMotor.resetAndShutdown();
    isDirectionForward = true;
}

/*
 * This stops motors
 */
void CarMotorControl::setDirection(bool goForward) {
    isDirectionForward = goForward;
    rightEncoderMotor.setDirection(goForward);
    leftEncoderMotor.setDirection(goForward);
}

/*
 * initialize motorInfo fields DirectionForward, ActualMaxSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void CarMotorControl::initGoDistanceCentimeter(int aDistanceCentimeter) {
    rightEncoderMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT);
    leftEncoderMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT);
}

/**
 * Wait until distance is reached
 * @param  aLoopCallback called until car has stopped to avoid blocking
 *
 */
void CarMotorControl::goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void)) {
    initGoDistanceCentimeter(aDistanceCentimeter);
    waitUntilCarStopped(aLoopCallback);
}

/*
 * Start motor for "infinite" distance and then blocking wait until both motors are at full speed
 */
void CarMotorControl::startAndWaitForFullSpeed() {
    initGoDistanceCentimeter(3200);
    /*
     * blocking wait for start
     */
    do {
        rightEncoderMotor.updateMotor();
        leftEncoderMotor.updateMotor();
    } while (rightEncoderMotor.State != MOTOR_STATE_FULL_SPEED || leftEncoderMotor.State != MOTOR_STATE_FULL_SPEED);
}


/*
 * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_FULL_SPEED to MOTOR_STATE_RAMP_DOWN
 * Use DistanceCountAfterRampUp as ramp down count
 * Blocking wait for stop
 */
void CarMotorControl::stopCar() {
    if (isStopped()) {
        return;
    }
    rightEncoderMotor.NextChangeMaxTargetCount = rightEncoderMotor.DistanceCount;
    rightEncoderMotor.TargetDistanceCount = rightEncoderMotor.DistanceCount + rightEncoderMotor.DistanceCountAfterRampUp;
    leftEncoderMotor.NextChangeMaxTargetCount = leftEncoderMotor.DistanceCount;
    leftEncoderMotor.TargetDistanceCount = leftEncoderMotor.DistanceCount + leftEncoderMotor.DistanceCountAfterRampUp;

    /*
     * blocking wait for stop
     */
    do {
        rightEncoderMotor.updateMotor();
        leftEncoderMotor.updateMotor();
    } while (!isStopped());
}

/*
 * Non blocking version of wait
 */
void CarMotorControl::waitUntilCarStopped(void (*aLoopCallback)(void)) {
    do {
        rightEncoderMotor.updateMotor();
        leftEncoderMotor.updateMotor();
        if (aLoopCallback != NULL) {
            aLoopCallback();
        }
    } while (!isStopped());
}

/**
 * Blocking version of wait
 */
void CarMotorControl::waitUntilCarStopped() {
    do {
        rightEncoderMotor.updateMotor();
        leftEncoderMotor.updateMotor();
    } while (!isStopped());
}

bool CarMotorControl::isState(uint8_t aState) {
    return (rightEncoderMotor.State == aState && leftEncoderMotor.State == aState);
}

bool CarMotorControl::isStopped() {
    return isState(MOTOR_STATE_STOPPED);
}

bool CarMotorControl::isStarted() {
    return isState(MOTOR_STATE_FULL_SPEED);
}

/**
 * Init motors for turn and wait for car to stop
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 */
void CarMotorControl::rotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed) {
    if (aRotationDegrees != 0) {
        initRotateCar(aRotationDegrees, aTurnDirection, aUseSlowSpeed);
        waitUntilCarStopped();
    }
}

/**
 * Set distances and speed for 2 motors to turn the requested angle
 * @param  if aUseSlowSpeed is true then use slower speed (1.5 times MinSpeed) for rotation to be more exact
 */
void CarMotorControl::initRotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed) {
    int tDistanceCountRight;
    int tDistanceCountLeft;
    float tFactor;
    if (is2WDCar) {
        tFactor = FACTOR_DEGREE_TO_COUNT_2WD_CAR;
    } else {
        tFactor = FACTOR_DEGREE_TO_COUNT_4WD_CAR;
    }
    if (aRotationDegrees > 0) {
        // turn left, default is TURN_FORWARD
        tDistanceCountRight = (aRotationDegrees * tFactor) + 0.5;
        tDistanceCountLeft = 0;
        if (aTurnDirection == TURN_IN_PLACE) {
            tDistanceCountLeft = -tDistanceCountRight / 2;
            tDistanceCountRight = tDistanceCountRight / 2;
        }
    } else {
        // turn right, default is TURN_FORWARD
        tDistanceCountLeft = (-aRotationDegrees * tFactor) + 0.5;
        tDistanceCountRight = 0;
        if (aTurnDirection == TURN_IN_PLACE) {
            tDistanceCountRight = -tDistanceCountLeft / 2;
            tDistanceCountLeft = tDistanceCountLeft / 2;
        }
    }
    if (aTurnDirection == TURN_BACKWARD) {
        // swap motors
        int tDistanceCountTemp = tDistanceCountRight;
        tDistanceCountRight = -tDistanceCountLeft;
        tDistanceCountLeft = -tDistanceCountTemp;

    }
    // This in turn sets ActualMaxSpeed to MaxSpeed.
    rightEncoderMotor.initGoDistanceCount(tDistanceCountRight);
    leftEncoderMotor.initGoDistanceCount(tDistanceCountLeft);
    if (aUseSlowSpeed) {
        // adjust MaxSpeed
        rightEncoderMotor.ActualMaxSpeed = rightEncoderMotor.MinSpeed + rightEncoderMotor.MinSpeed / 2;
        leftEncoderMotor.ActualMaxSpeed = leftEncoderMotor.MinSpeed + leftEncoderMotor.MinSpeed / 2;
    }
}

/**
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 * @param  aLoopCallback avoid blocking and call aLoopCallback on waiting for stop
 */
void CarMotorControl::rotateCar(int16_t aRotationDegrees, void (*aLoopCallback)(void), uint8_t aTurnDirection, bool aUseSlowSpeed) {
    if (aRotationDegrees != 0) {
        initRotateCar(aRotationDegrees, aTurnDirection, aUseSlowSpeed);
        waitUntilCarStopped(aLoopCallback);
    }
}

// ISR for PIN PD2 / RIGHT
ISR(INT0_vect) {
    rightEncoderMotor.handleEncoderInterrupt();
}

// ISR for PIN PD3 / LEFT
ISR(INT1_vect) {
    leftEncoderMotor.handleEncoderInterrupt();
}

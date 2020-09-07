/*
 * CarMotorControl.cpp
 *
 *  Contains functions for control of the 2 motors of a car like setDirection, goDistanceCentimeter() and rotateCar().
 *  Checks input of PIN aPinFor2WDDetection since we need different factors for rotating a 4 wheel and a 2 wheel car.
 *
 *  Requires EncoderMotor.cpp
 *
 *  Created on: 16.09.2016
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

#include <Arduino.h>
#include "CarMotorControl.h"
#include "EncoderMotor.h"

EncoderMotor rightEncoderMotor;
EncoderMotor leftEncoderMotor;

CarMotorControl::CarMotorControl() { // @suppress("Class members should be properly initialized")
}

#ifdef USE_TB6612_BREAKOUT_BOARD
void CarMotorControl::init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin,
        uint8_t aLeftMotorForwardPin, uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin, uint8_t aPinFor2WDDetection) {
    pinMode(aPinFor2WDDetection, INPUT_PULLUP);

    leftEncoderMotor.init(aLeftMotorForwardPin, LeftMotorBackwardPin, aLeftMotorPWMPin);
    rightEncoderMotor.init(aRightMotorForwardPin, aRightMotorBackwardPin, aRightPWMPin);
    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    EncoderMotor::enableINT0AndINT1Interrupts();

    is2WDCar = !digitalRead(aPinFor2WDDetection);
}
#else
void CarMotorControl::init(uint8_t aPinFor2WDDetection) {
    pinMode(aPinFor2WDDetection, INPUT_PULLUP);

    leftEncoderMotor.init(0);
    rightEncoderMotor.init(1);
    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    EncoderMotor::enableINT0AndINT1Interrupts();

    is2WDCar = !digitalRead(aPinFor2WDDetection);
}
#endif

/*
 * Sets default values for min and max speed and reset compensation
 */
void CarMotorControl::setDefaultsForFixedDistanceDriving() {
    rightEncoderMotor.setEepromValuesDefaults();
    leftEncoderMotor.setEepromValuesDefaults();
}

/*
 *  Direct motor control, no state or flag handling
 */
void CarMotorControl::setSpeed(int aRequestedSpeed) {
    rightEncoderMotor.setSpeed(aRequestedSpeed);
    leftEncoderMotor.setSpeed(aRequestedSpeed);
}

/*
 * Sets signed speed adjusted by current compensation value and handle motor state and flags
 * Not used yet
 */
void CarMotorControl::setCurrentSpeedCompensated(int aRequestedSpeed) {
    rightEncoderMotor.setCurrentSpeedCompensated(aRequestedSpeed);
    leftEncoderMotor.setCurrentSpeedCompensated(aRequestedSpeed);
}

/*
 * Sets speed adjusted by current compensation value and handle motor state and flags
 */
void CarMotorControl::setCurrentSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    rightEncoderMotor.setCurrentSpeedCompensated(aRequestedSpeed, aRequestedDirection);
    leftEncoderMotor.setCurrentSpeedCompensated(aRequestedSpeed, aRequestedDirection);
}

/*
 * Stop car
 */
void CarMotorControl::stopMotors(uint8_t aStopMode) {
    rightEncoderMotor.stopMotor(aStopMode);
    leftEncoderMotor.stopMotor(aStopMode);
}

/*
 * Stop car and reset all control values as speed, distances, debug values to 0x00
 * Leave calibration and compensation (EEPROM) values unaffected.
 */
void CarMotorControl::stopMotorsAndReset() {
    rightEncoderMotor.stopMotorAndReset();
    leftEncoderMotor.stopMotorAndReset();
}

/*
 * If motor is accelerating or decelerating then updateMotor needs to be called at a fast rate otherwise it will not work correctly
 * Used to suppress time consuming display of motor values
 */
bool CarMotorControl::needsFastUpdates() {
    return (rightEncoderMotor.State == MOTOR_STATE_RAMP_DOWN || rightEncoderMotor.State == MOTOR_STATE_RAMP_UP
            || leftEncoderMotor.State == MOTOR_STATE_RAMP_DOWN || leftEncoderMotor.State == MOTOR_STATE_RAMP_UP);
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool CarMotorControl::updateMotors() {
    bool tMotorsNotStopped = rightEncoderMotor.updateMotor();
    tMotorsNotStopped |= leftEncoderMotor.updateMotor();
    return tMotorsNotStopped;
}

/*
 * initialize motorInfo fields DirectionForward, CurrentMaxSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void CarMotorControl::initGoDistanceCentimeter(int aDistanceCentimeter) {
    rightEncoderMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT);
    leftEncoderMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT);
}

void CarMotorControl::initGoDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection) {
    rightEncoderMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT, aRequestedDirection);
    leftEncoderMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT, aRequestedDirection);
}

/**
 * Wait until distance is reached
 * @param  aLoopCallback called until car has stopped to avoid blocking
 */
void CarMotorControl::goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void)) {
    initGoDistanceCentimeter(aDistanceCentimeter);
    waitUntilCarStopped(aLoopCallback);
}

void CarMotorControl::goDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection,
        void (*aLoopCallback)(void)) {
    initGoDistanceCentimeter(aDistanceCentimeter, aRequestedDirection);
    waitUntilCarStopped(aLoopCallback);
}
/*
 * Start motor for "infinite" distance and then blocking wait until both motors are at full speed
 * If motor is still running, just update motor.
 */
void CarMotorControl::startCarAndWaitForFullSpeed(uint8_t aDriveDirection) {
    if (rightEncoderMotor.State == MOTOR_STATE_STOPPED || leftEncoderMotor.State == MOTOR_STATE_STOPPED
            || rightEncoderMotor.CurrentDirection != aDriveDirection) {
        /*
         * Start only if not already started or direction changed
         */
        if (aDriveDirection == DIRECTION_FORWARD) {
            initGoDistanceCentimeter(INFINITE_DISTANCE_CM);
        } else {
            initGoDistanceCentimeter(-INFINITE_DISTANCE_CM);
        }
#ifdef DEBUG
        Serial.print(F("Start car dir="));
        Serial.println(aDriveDirection);
#endif
    }
    /*
     * blocking wait for start
     */
    bool tMotorsNotStopped;
    do {
        tMotorsNotStopped = rightEncoderMotor.updateMotor();
        tMotorsNotStopped |= leftEncoderMotor.updateMotor();
    } while (tMotorsNotStopped
            && (rightEncoderMotor.State != MOTOR_STATE_FULL_SPEED || leftEncoderMotor.State != MOTOR_STATE_FULL_SPEED));
}

/*
 * Stop car with ramp and give DistanceCountAfterRampUp counts for braking.
 *
 * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_FULL_SPEED to MOTOR_STATE_RAMP_DOWN
 * Use DistanceCountAfterRampUp as ramp down count
 * Blocking wait for stop
 */
void CarMotorControl::stopCarAndWaitForIt() {
    if (isStopped()) {
        return;
    }
    rightEncoderMotor.NextChangeMaxTargetCount = rightEncoderMotor.EncoderCount;
    rightEncoderMotor.TargetDistanceCount = rightEncoderMotor.EncoderCount + rightEncoderMotor.DistanceCountAfterRampUp;
    leftEncoderMotor.NextChangeMaxTargetCount = leftEncoderMotor.EncoderCount;
    leftEncoderMotor.TargetDistanceCount = leftEncoderMotor.EncoderCount + leftEncoderMotor.DistanceCountAfterRampUp;

    /*
     * blocking wait for stop
     */
    waitUntilCarStopped();
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
        // turn left, compute values for TURN_FORWARD
        tDistanceCountRight = (aRotationDegrees * tFactor) + 0.5;
        tDistanceCountLeft = 0;
        if (aTurnDirection == TURN_IN_PLACE) {
            tDistanceCountRight /= 2;
            tDistanceCountLeft = -tDistanceCountRight;
        }
    } else {
        // turn right, compute values for TURN_FORWARD
        tDistanceCountLeft = (-aRotationDegrees * tFactor) + 0.5;
        tDistanceCountRight = 0;
        if (aTurnDirection == TURN_IN_PLACE) {
            tDistanceCountLeft /= 2;
            tDistanceCountRight = -tDistanceCountLeft;
        }
    }
    if (aTurnDirection == TURN_BACKWARD) {
        // swap motors and sign
        int tDistanceCountTemp = tDistanceCountRight;
        tDistanceCountRight = -tDistanceCountLeft;
        tDistanceCountLeft = -tDistanceCountTemp;

    }
    // This in turn sets CurrentMaxSpeed to MaxSpeed.
    rightEncoderMotor.initGoDistanceCount(tDistanceCountRight);
    leftEncoderMotor.initGoDistanceCount(tDistanceCountLeft);
    if (aUseSlowSpeed) {
        // adjust MaxSpeed
        rightEncoderMotor.CurrentMaxSpeed = rightEncoderMotor.MinSpeed + rightEncoderMotor.MinSpeed / 2;
        leftEncoderMotor.CurrentMaxSpeed = leftEncoderMotor.MinSpeed + leftEncoderMotor.MinSpeed / 2;
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
/*
 * PWMDcMotor.cpp
 *
 * Low level motor control for Adafruit_MotorShield OR breakout board with TB6612 or L298 driver IC for two DC motors.
 *
 * Motor control has 2 parameters:
 * 1. SpeedPWM / PWM which is ignored for BRAKE or RELEASE. This library also accepts signed speed (including the direction as sign).
 * 2. Direction / MotorDriverMode. Can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 *
 * PWM period is 600 us for Adafruit Motor Shield V2 using PCA9685.
 * PWM period is 1030 us for using AnalogWrite on pin 5 + 6.
 *
 *  Created on: 12.05.2019
 *  Copyright (C) 2019-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of PWMMotorControl https://github.com/ArminJo/PWMMotorControl.
 *
 *  PWMMotorControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
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

#include "PWMDcMotor.h"

//#define TRACE
//#define DEBUG

// Flags e.g. for display update control
#if defined(USE_MPU6050_IMU) || defined(USE_ENCODER_MOTOR_CONTROL)
volatile bool PWMDcMotor::SensorValuesHaveChanged; // true if encoder count and derived encoder speed, or one of the TMU data have changed
#endif
bool PWMDcMotor::MotorControlValuesHaveChanged; // true if DefaultStopMode, StartSpeedPWM, DriveSpeedPWM or SpeedPWMCompensation have changed
bool PWMDcMotor::MotorPWMHasChanged;              // true if CurrentSpeedPWM has changed

PWMDcMotor::PWMDcMotor() { // @suppress("Class members should be properly initialized")
}

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
void PWMDcMotor::PCA9685WriteByte(uint8_t aAddress, uint8_t aData) {
    Wire.beginTransmission(PCA9685_DEFAULT_ADDRESS);
    Wire.write(aAddress);
    Wire.write(aData);
    Wire.endTransmission(true);
}

void PWMDcMotor::PCA9685SetPWM(uint8_t aPin, uint16_t aOn, uint16_t aOff) {
    Wire.beginTransmission(PCA9685_DEFAULT_ADDRESS);
    Wire.write((PCA9685_FIRST_PWM_REGISTER) + 4 * aPin);
    Wire.write(aOn);
    Wire.write(aOn >> 8);
    Wire.write(aOff);
    Wire.write(aOff >> 8);
    Wire.endTransmission(true);
}

void PWMDcMotor::PCA9685SetPin(uint8_t aPin, bool aSetToOn) {
    if (aSetToOn) {
        PCA9685SetPWM(aPin, 4096, 0);
    } else {
        PCA9685SetPWM(aPin, 0, 0);
    }
}

#  else
// Create the motor shield object with the default I2C address
Adafruit_MotorShield sAdafruitMotorShield = Adafruit_MotorShield();
#  endif // USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD

/*
 * aMotorNumber from 1 to 2
 * Currently motors 3 and 4 are not required/supported by own library for Adafruit Motor Shield
 */
void PWMDcMotor::init(uint8_t aMotorNumber) {
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    if (aMotorNumber == 1) {
        // Set PCA9685 channel numbers for Adafruit Motor Shield
        PWMPin = 8;
        BackwardPin = 9;
        ForwardPin = 10;
    } else {
        PWMPin = 13;
        BackwardPin = 12;
        ForwardPin = 11;
    }

    Wire.begin();
    Wire.setClock(400000);
#    if defined (ARDUINO_ARCH_AVR) // Other platforms do not have this new function
    Wire.setWireTimeout(5000); // Sets timeout to 5 ms. default is 25 ms.
#    endif
#ifdef TRACE
    Serial.print(PWMPin);
    Serial.print(F(" MotorNumber="));
    Serial.println(aMotorNumber);
#endif
    // Reset PCA9685
    Wire.beginTransmission(PCA9685_GENERAL_CALL_ADDRESS);
    Wire.write(PCA9685_SOFTWARE_RESET);
    Wire.endTransmission(true);
    // Set expander to 1600 HZ
    PCA9685WriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_MODE_1_SLEEP)); // go to sleep
    PCA9685WriteByte(PCA9685_PRESCALE_REGISTER, PCA9685_PRESCALER_FOR_1600_HZ); // set the prescaler
    delay(2); // > 500 us before the restart bit according to datasheet
    PCA9685WriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_MODE_1_RESTART) | _BV(PCA9685_MODE_1_AUTOINCREMENT)); // reset sleep and enable auto increment

#  else
    Adafruit_MotorShield_DcMotor = sAdafruitMotorShield.getMotor(aMotorNumber);
    sAdafruitMotorShield.begin();
#  endif

    setDefaultsForFixedDistanceDriving();
    stop(DEFAULT_STOP_MODE);
}

#else // USE_ADAFRUIT_MOTOR_SHIELD
void PWMDcMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin) {
    ForwardPin = aForwardPin;
    BackwardPin = aBackwardPin;
    PWMPin = aPWMPin;
    DefaultStopMode = MOTOR_RELEASE;

    pinMode(aForwardPin, OUTPUT);
    pinMode(aBackwardPin, OUTPUT);
    pinMode(aPWMPin, OUTPUT);

    // set defaults
    setDefaultsForFixedDistanceDriving();
    stop(DEFAULT_STOP_MODE);
}

#endif // USE_ADAFRUIT_MOTOR_SHIELD

/*
 *  @brief  Control the DC motor driver direction and stop mode
 *  @param  aMotorDriverMode The mode can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 */
void PWMDcMotor::setMotorDriverMode(uint8_t aMotorDriverMode) {
    CurrentDirectionOrBrakeMode = aMotorDriverMode; // The only statement which changes CurrentDirectionOrBrakeMode
    if (!(aMotorDriverMode & STOP_MODE_OR_MASK)) {
        // set only directions, no brake modes
        LastDirection = aMotorDriverMode;
    }
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    // until here DIRECTION_FORWARD is 0 back is 1, Adafruit library starts with 1
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    switch (aMotorDriverMode) {
    case DIRECTION_FORWARD:
        PCA9685SetPin(BackwardPin, LOW); // take low first to avoid 'break'
        PCA9685SetPin(ForwardPin, HIGH);
        break;
    case DIRECTION_BACKWARD:
        PCA9685SetPin(ForwardPin, LOW); // take low first to avoid 'break'
        PCA9685SetPin(BackwardPin, HIGH);
        break;
    case MOTOR_BRAKE:
        PCA9685SetPin(ForwardPin, HIGH);
        PCA9685SetPin(BackwardPin, HIGH);
        break;
    case MOTOR_RELEASE:
        PCA9685SetPin(ForwardPin, LOW);
        PCA9685SetPin(BackwardPin, LOW);
        break;
    }
#  else
    Adafruit_MotorShield_DcMotor->run(aMotorDriverMode + CONVERSION_FOR_ADAFRUIT_API);
#  endif

#else
    switch (aMotorDriverMode) {
    case DIRECTION_FORWARD:
        digitalWrite(BackwardPin, LOW); // take low first to avoid 'break'
        digitalWrite(ForwardPin, HIGH);
        break;
    case DIRECTION_BACKWARD:
        digitalWrite(ForwardPin, LOW); // take low first to avoid 'break'
        digitalWrite(BackwardPin, HIGH);
        break;
    case MOTOR_BRAKE:
        digitalWrite(ForwardPin, HIGH);
        digitalWrite(BackwardPin, HIGH);
        break;
    case MOTOR_RELEASE:
        digitalWrite(ForwardPin, LOW);
        digitalWrite(BackwardPin, LOW);
        break;
    }
#endif // USE_ADAFRUIT_MOTOR_SHIELD
}

/*
 * @return true if direction has changed AND motor was stopped
 */
bool PWMDcMotor::checkAndHandleDirectionChange(uint8_t aRequestedDirection) {
    aRequestedDirection &= DIRECTION_MASK; // since we are never called with "brake directions" but may be called with TURN_IN_PLACE direction
    bool tReturnValue = false;
    if (CurrentDirectionOrBrakeMode != aRequestedDirection) {
        if (CurrentSpeedPWM != 0) {
            /*
             * Direction change requested but motor still running-> first stop motor
             */
            stop(MOTOR_BRAKE);
            tReturnValue = true;
        }
#ifdef DEBUG
        Serial.print(PWMPin);
        Serial.print(F(" Change motor mode from "));
        Serial.print(CurrentDirectionOrBrakeMode);
        Serial.print(F(" to "));
        Serial.println(aRequestedDirection);
#endif
        setMotorDriverMode(aRequestedDirection); // this in turn sets CurrentDirectionOrBrakeMode
    }
    return tReturnValue;
}

/*
 *  @brief  Control the DC Motor speed/throttle
 *  @param  speed The 8-bit PWM value, 0 is off, 255 is on forward -255 is on backward
 *  First set driver mode, then set PWM
 *  PWM period is 600 us for Adafruit Motor Shield V2 using PCA9685.
 *  PWM period is 1030 us for using AnalogWrite on pin 5 + 6.
 */
void PWMDcMotor::setSpeedPWM(uint8_t aSpeedPWMRequested, uint8_t aRequestedDirection) {
    if (aSpeedPWMRequested == 0) {
        stop(DefaultStopMode);
    } else {
        checkAndHandleDirectionChange(aRequestedDirection);
        if (CurrentSpeedPWM != aSpeedPWMRequested) {
            CurrentSpeedPWM = aSpeedPWMRequested; // The only statement which sets CurrentSpeedPWM to a value != 0
#ifdef TRACE
            Serial.print(PWMPin);
            Serial.print(F(" SpeedPWM="));
            Serial.println(CurrentSpeedPWM);
#endif
            MotorPWMHasChanged = true;
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
            PCA9685SetPWM(PWMPin, 0, 16 * aSpeedPWMRequested);
#  else
            Adafruit_MotorShield_DcMotor->setSpeedPWM(aSpeedPWMRequested);
#  endif
#else
            analogWrite(PWMPin, aSpeedPWMRequested);
#endif
        }
    }
}

/*
 * Keeps direction
 */
void PWMDcMotor::changeSpeedPWM(uint8_t aSpeedPWMRequested) {
    if (!(CurrentDirectionOrBrakeMode & STOP_MODE_OR_MASK)) {
        setSpeedPWM(aSpeedPWMRequested, CurrentDirectionOrBrakeMode); // output PWM value to motor
    }
}

/*
 * Subtracts SpeedPWMCompensation from aRequestedSpeedPWM before applying
 */
void PWMDcMotor::setSpeedPWMCompensated(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
    // avoid underflow
    uint8_t tCurrentSpeedPWM;
    if (aRequestedSpeedPWM > SpeedPWMCompensation) {
        tCurrentSpeedPWM = aRequestedSpeedPWM - SpeedPWMCompensation;
    } else {
        tCurrentSpeedPWM = 0;
    }
    setSpeedPWM(tCurrentSpeedPWM, aRequestedDirection); // output PWM value to motor
}

/*
 * Keeps direction
 */
void PWMDcMotor::changeSpeedPWMCompensated(uint8_t aRequestedSpeedPWM) {
    if (!(CurrentDirectionOrBrakeMode & STOP_MODE_OR_MASK)) {
        setSpeedPWMCompensated(aRequestedSpeedPWM, CurrentDirectionOrBrakeMode);
    }
}

/*
 * Signed speed
 */
void PWMDcMotor::setSpeedPWM(int aSpeedPWMRequested) {
    if (aSpeedPWMRequested < 0) {
        aSpeedPWMRequested = -aSpeedPWMRequested;
        setSpeedPWM(aSpeedPWMRequested, DIRECTION_BACKWARD);
    } else {
        setSpeedPWM(aSpeedPWMRequested, DIRECTION_FORWARD);
    }
}

void PWMDcMotor::setSpeedPWMCompensated(int aRequestedSpeedPWM) {
    uint8_t tDirection;
    if (aRequestedSpeedPWM > 0) {
        tDirection = DIRECTION_FORWARD;
    } else {
        tDirection = DIRECTION_BACKWARD;
        aRequestedSpeedPWM = -aRequestedSpeedPWM;
    }
    setSpeedPWMCompensated(aRequestedSpeedPWM, tDirection);
}

/*
 * First set PWM to 0 then disable driver
 * @param aStopMode STOP_MODE_KEEP (take previously defined DefaultStopMode) or MOTOR_BRAKE or MOTOR_RELEASE
 */
void PWMDcMotor::stop(uint8_t aStopMode) {

    CurrentSpeedPWM = 0; // The only statement which sets CurrentSpeedPWM to 0
    MotorPWMHasChanged = true;
    CheckDistanceInUpdateMotor = false;
    MotorRampState = MOTOR_STATE_STOPPED;

#ifndef USE_ENCODER_MOTOR_CONTROL
    CheckDistanceInUpdateMotor = false;
#endif
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    PCA9685SetPWM(PWMPin, 0, 0);
#  else
        Adafruit_MotorShield_DcMotor->setSpeedPWM(0);
#  endif
#else
    analogWrite(PWMPin, 0);
#endif
    if (aStopMode == STOP_MODE_KEEP) {
        aStopMode = DefaultStopMode;
    }
    setMotorDriverMode(ForceStopMODE(aStopMode));
#ifdef DEBUG
    Serial.print(PWMPin);
    Serial.print(F(" Stop motor StopMode="));
    Serial.println(ForceStopMODE(aStopMode));
#endif
}

/*
 * @param aStopMode used for speed == 0 or STOP_MODE_KEEP: MOTOR_BRAKE or MOTOR_RELEASE
 */
void PWMDcMotor::setStopMode(uint8_t aStopMode) {
    DefaultStopMode = ForceStopMODE(aStopMode);
    MotorControlValuesHaveChanged = true;
}

/******************************************************************************************
 * Distance functions
 *****************************************************************************************/
/*
 * setDefaultsForFixedDistanceDriving() is called at init
 */
void PWMDcMotor::setDefaultsForFixedDistanceDriving() {
    StartSpeedPWM = DEFAULT_START_SPEED_PWM;
    DriveSpeedPWM = DEFAULT_DRIVE_SPEED_PWM;
    SpeedPWMCompensation = 0;
#ifndef USE_ENCODER_MOTOR_CONTROL
    MillisPerMillimeter = DEFAULT_MILLIS_PER_MILLIMETER;
#endif
    MotorControlValuesHaveChanged = true;
}

void PWMDcMotor::setValuesForFixedDistanceDriving(uint8_t aStartSpeedPWM, uint8_t aDriveSpeedPWM, uint8_t aSpeedPWMCompensation) {
    StartSpeedPWM = aStartSpeedPWM;
    DriveSpeedPWM = aDriveSpeedPWM;
    SpeedPWMCompensation = aSpeedPWMCompensation;
    MotorControlValuesHaveChanged = true;
}

void PWMDcMotor::setSpeedPWMCompensation(uint8_t aSpeedPWMCompensation) {
    SpeedPWMCompensation = aSpeedPWMCompensation;
    MotorControlValuesHaveChanged = true;
}

void PWMDcMotor::setStartSpeedPWM(uint8_t aStartSpeedPWM) {
    StartSpeedPWM = aStartSpeedPWM;
    MotorControlValuesHaveChanged = true;
}

void PWMDcMotor::setDriveSpeedPWM(uint8_t aDriveSpeedPWM) {
    DriveSpeedPWM = aDriveSpeedPWM;
    MotorControlValuesHaveChanged = true;
}

void PWMDcMotor::startRampUp(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
#ifdef DEBUG
    Serial.print(PWMPin);
    Serial.print(F(" Ramp up to "));
    Serial.print(aRequestedSpeedPWM);
    Serial.print(F(" Dir="));
    Serial.print(aRequestedDirection);
    Serial.print(F(" CurrentSpeedPWM="));
    Serial.print(CurrentSpeedPWM);
    Serial.print(F(" MotorMode="));
    Serial.print(CurrentDirectionOrBrakeMode);
    Serial.println();
#endif

    if (CurrentSpeedPWM == 0) { // equivalent to MotorRampState == MOTOR_STATE_STOPPED
        checkAndHandleDirectionChange(aRequestedDirection);
        MotorRampState = MOTOR_STATE_START;
        /*
         * Set target speed for ramp up
         */
        uint8_t tRequestedDriveSpeedPWM;
        if (aRequestedSpeedPWM > SpeedPWMCompensation) {
            tRequestedDriveSpeedPWM = aRequestedSpeedPWM - SpeedPWMCompensation;
        } else {
            tRequestedDriveSpeedPWM = 0;
        }
        RequestedDriveSpeedPWM = tRequestedDriveSpeedPWM;
        RampSpeedPWMDelta = RAMP_UP_VALUE_DELTA; // 20V / s
    } else if (MotorRampState == MOTOR_STATE_DRIVE) {
        // motor is running, -> just change speed
        setSpeedPWMCompensated(aRequestedSpeedPWM, aRequestedDirection);
    }
    // else ramp is in mode MOTOR_STATE_RAMP_UP -> do nothing, let the ramp go on
#  ifdef DEBUG
    Serial.print(F("MotorRampState="));
    Serial.print(MotorRampState);
    Serial.println();
#  endif
}

void PWMDcMotor::startRampUp(uint8_t aRequestedDirection) {
    startRampUp(DriveSpeedPWM, aRequestedDirection);
}

void PWMDcMotor::startRampDown() {
    MotorRampState = MOTOR_STATE_RAMP_DOWN;
// set only the variables for later evaluation in updateMotor() below
    if (CurrentSpeedPWM > (RAMP_VALUE_DOWN_OFFSET_SPEED_PWM - RAMP_DOWN_VALUE_DELTA)) {
        CurrentSpeedPWM -= (RAMP_VALUE_DOWN_OFFSET_SPEED_PWM - RAMP_DOWN_VALUE_DELTA); // RAMP_DOWN_VALUE_DELTA is immediately subtracted below
    } else {
        CurrentSpeedPWM = RAMP_VALUE_DOWN_MIN_SPEED_PWM;
    }
}

#if !defined(USE_ENCODER_MOTOR_CONTROL)
/*
 * @return true if not stopped (motor expects another update)
 */
bool PWMDcMotor::updateMotor() {
    unsigned long tMillis = millis();
    uint8_t tNewSpeedPWM = CurrentSpeedPWM;

    if (MotorRampState == MOTOR_STATE_START) {
        NextRampChangeMillis = tMillis + RAMP_INTERVAL_MILLIS;
        /*
         * Start motor
         */
        if (RequestedDriveSpeedPWM > RAMP_VALUE_UP_OFFSET_SPEED_PWM) {
            // start with ramp to avoid spinning wheels
            tNewSpeedPWM = RAMP_VALUE_UP_OFFSET_SPEED_PWM; // start immediately with speed offset (3 volt)
            //  --> RAMP_UP
            MotorRampState = MOTOR_STATE_RAMP_UP;
        } else {
            // Motor ramp not required, go direct to drive speed.
            tNewSpeedPWM = RequestedDriveSpeedPWM;
            //  --> DRIVE
            MotorRampState = MOTOR_STATE_DRIVE;
        }
    } else if (MotorRampState == MOTOR_STATE_RAMP_UP) {
        /*
         * Increase motor speed by RAMP_UP_VALUE_DELTA every RAMP_UP_UPDATE_INTERVAL_MILLIS milliseconds
         */
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis += RAMP_INTERVAL_MILLIS;
            tNewSpeedPWM = tNewSpeedPWM + RampSpeedPWMDelta;
            /*
             * Transition criteria is: RequestedDriveSpeedPWM reached.
             * Then check immediately for timeout
             */
            // Clip value and check for 8 bit overflow
            if (tNewSpeedPWM >= RequestedDriveSpeedPWM || tNewSpeedPWM <= RampSpeedPWMDelta) {
                tNewSpeedPWM = RequestedDriveSpeedPWM;
                //  --> DRIVE
                MotorRampState = MOTOR_STATE_DRIVE;
            }
        }
    }

    if (MotorRampState == MOTOR_STATE_DRIVE) {
        /*
         * Time based distance. Ramp down is included in time formula.
         */
        if (CheckDistanceInUpdateMotor && millis() >= computedMillisOfMotorStopForDistance) {

            if (tNewSpeedPWM > (RAMP_VALUE_DOWN_OFFSET_SPEED_PWM - RAMP_DOWN_VALUE_DELTA)) {
                tNewSpeedPWM -= (RAMP_VALUE_DOWN_OFFSET_SPEED_PWM - RAMP_DOWN_VALUE_DELTA); // RAMP_DOWN_VALUE_DELTA is immediately subtracted below
                //  --> RAMP_DOWN
                MotorRampState = MOTOR_STATE_RAMP_DOWN;
            } else {
                //  --> STOPPED
                tNewSpeedPWM = 0;
            }
        }
    }

    if (MotorRampState == MOTOR_STATE_RAMP_DOWN) {
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis = tMillis + RAMP_INTERVAL_MILLIS;
            /*
             * Decrease motor speed RAMP_UP_UPDATE_INTERVAL_STEPS times every RAMP_UP_UPDATE_INTERVAL_MILLIS milliseconds
             */
            if (tNewSpeedPWM > (RAMP_DOWN_VALUE_DELTA + RAMP_VALUE_DOWN_MIN_SPEED_PWM)) {
                tNewSpeedPWM -= RAMP_DOWN_VALUE_DELTA;
            } else {
                tNewSpeedPWM = RAMP_VALUE_DOWN_MIN_SPEED_PWM;
            }
        }
    }
// End of motor state machine

#ifdef TRACE
        Serial.print(F("St="));
        Serial.println(MotorRampState);
#endif
    if (tNewSpeedPWM != CurrentSpeedPWM) {
#ifdef TRACE
        Serial.print(F("Ns="));
        Serial.println(tNewSpeedPWM);
#endif
        PWMDcMotor::setSpeedPWM(tNewSpeedPWM, CurrentDirectionOrBrakeMode); // sets MOTOR_STATE_STOPPED if speed is 0
    }

    /*
     * Check if target milliseconds are reached
     */
    if (CurrentSpeedPWM > 0) {
        if (CheckDistanceInUpdateMotor && millis() > computedMillisOfMotorStopForDistance) {
            stop(DefaultStopMode); // resets CheckDistanceInUpdateMotor and sets MOTOR_STATE_STOPPED;
            return false; // need no more calls to update()
        }
        return true; // still running
    }
    return false; // current speed == 0
}

/*
 * Required for non encoder motors to estimate duration for a fixed distance
 */
void PWMDcMotor::setMillimeterPerSecondForFixedDistanceDriving(uint16_t aMillimeterPerSecond) {
    MillisPerMillimeter = 1000 / aMillimeterPerSecond;
}

/*
 * If motor is already running just update speed and new time
 */
void PWMDcMotor::startGoDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {
    if (aRequestedDistanceMillimeter == 0) {
        stop(DefaultStopMode); // In case motor was running
        return;
    }

    if (CurrentSpeedPWM == 0) {
        startRampUp(aRequestedSpeedPWM, aRequestedDirection);

        /*
         * Estimate duration for given distance#
         * MillisPerMillimeter is defined for DEFAULT_DRIVE_SPEED_PWM is
         */
        if (aRequestedSpeedPWM == DEFAULT_DRIVE_SPEED_PWM) {
            computedMillisOfMotorStopForDistance = millis() + DEFAULT_MOTOR_START_UP_TIME_MILLIS
                    + (((uint16_t) aRequestedDistanceMillimeter * MillisPerMillimeter));
        } else {
            computedMillisOfMotorStopForDistance = millis() + DEFAULT_MOTOR_START_UP_TIME_MILLIS
                    + (((uint32_t) aRequestedDistanceMillimeter * MillisPerMillimeter * DriveSpeedPWM) / DEFAULT_DRIVE_SPEED_PWM);
        }

    } else {
        MotorRampState = MOTOR_STATE_DRIVE;
        setSpeedPWMCompensated(aRequestedSpeedPWM, aRequestedDirection);
        /*
         * Estimate duration for given distance
         * use 32 bit intermediate to avoid overflow (this also saves around 50 bytes of program memory by using slower functions instead of faster inline code)
         */
        computedMillisOfMotorStopForDistance = millis()
                + (((uint32_t) aRequestedDistanceMillimeter * MillisPerMillimeter * DriveSpeedPWM) / DEFAULT_DRIVE_SPEED_PWM);
    }
    CheckDistanceInUpdateMotor = true;
}

void PWMDcMotor::startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection) {
    startGoDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
}

/*
 * Signed DistanceCount
 */
void PWMDcMotor::startGoDistanceMillimeter(int aRequestedDistanceMillimeter) {
    if (aRequestedDistanceMillimeter < 0) {
        aRequestedDistanceMillimeter = -aRequestedDistanceMillimeter;
        startGoDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, DIRECTION_BACKWARD);
    } else {
        startGoDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, DIRECTION_FORWARD);
    }
}

/*
 * Not used by CarControl
 */
void PWMDcMotor::goDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection) {
    goDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
}

void PWMDcMotor::goDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {
    startGoDistanceMillimeter(aRequestedSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
    while (millis() <= computedMillisOfMotorStopForDistance) {
        updateMotor();
    }
    stop(DefaultStopMode);
}

#endif // !defined(USE_ENCODER_MOTOR_CONTROL)

/********************************************************************************************
 * EEPROM functions
 * Uses the start of EEPROM for storage of EepromMotorInfoStruct's for motor number 1 to n
 ********************************************************************************************/
void PWMDcMotor::readMotorValuesFromEeprom(uint8_t aMotorValuesEepromStorageNumber) {
    EepromMotorInfoStruct tEepromMotorInfo;
    eeprom_read_block((void*) &tEepromMotorInfo, (void*) ((aMotorValuesEepromStorageNumber) * sizeof(EepromMotorInfoStruct)),
            sizeof(EepromMotorInfoStruct));

    /*
     * Overwrite with values if valid
     */
    if (tEepromMotorInfo.StartSpeedPWM < 150 && tEepromMotorInfo.StartSpeedPWM > 20) {
        StartSpeedPWM = tEepromMotorInfo.StartSpeedPWM;
        if (tEepromMotorInfo.DriveSpeedPWM > 40) {
            DriveSpeedPWM = tEepromMotorInfo.DriveSpeedPWM;
        }
        if (tEepromMotorInfo.SpeedPWMCompensation < 24) {
            SpeedPWMCompensation = tEepromMotorInfo.SpeedPWMCompensation;
        }
    }
    MotorControlValuesHaveChanged = true;
}

void PWMDcMotor::writeMotorValuesToEeprom(uint8_t aMotorValuesEepromStorageNumber) {
    EepromMotorInfoStruct tEepromMotorInfo;
    tEepromMotorInfo.StartSpeedPWM = StartSpeedPWM;
    tEepromMotorInfo.DriveSpeedPWM = DriveSpeedPWM;
    tEepromMotorInfo.SpeedPWMCompensation = SpeedPWMCompensation;

    eeprom_write_block((void*) &tEepromMotorInfo, (void*) ((aMotorValuesEepromStorageNumber) * sizeof(EepromMotorInfoStruct)),
            sizeof(EepromMotorInfoStruct));
}

const char StringNot[] PROGMEM = { " not" };
const char StringDefined[] PROGMEM = { " defined" };

void PWMDcMotor::printSettings(Print *aSerial) {
    aSerial->println();
    aSerial->println(F("Settings from PWMDcMotor.h:"));

    aSerial->print(F("USE_ENCODER_MOTOR_CONTROL:"));
#ifndef USE_ENCODER_MOTOR_CONTROL
    aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
#endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));

    aSerial->print(F("USE_ADAFRUIT_MOTOR_SHIELD:"));
#ifndef USE_ADAFRUIT_MOTOR_SHIELD
    aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
#endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    aSerial->print(F("USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD:"));
#ifndef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    aSerial->print(reinterpret_cast<const __FlashStringHelper *>(StringNot));
#endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));
#endif

    aSerial->print(F("DEFAULT_MOTOR_START_UP_TIME_MILLIS="));
    aSerial->println(DEFAULT_MOTOR_START_UP_TIME_MILLIS);

    aSerial->print(F("FULL_BRIDGE_OUTPUT_MILLIVOLT="));
    aSerial->print(FULL_BRIDGE_OUTPUT_MILLIVOLT);
    aSerial->print(F("mV (= FULL_BRIDGE_INPUT_MILLIVOLT|"));
    aSerial->print(FULL_BRIDGE_INPUT_MILLIVOLT);
    aSerial->print(F("mV - FULL_BRIDGE_LOSS_MILLIVOLT|"));
    aSerial->print(FULL_BRIDGE_LOSS_MILLIVOLT);
    aSerial->println(F("mV)"));

    aSerial->print(F("DEFAULT_START_SPEED_PWM="));
    aSerial->print(DEFAULT_START_SPEED_PWM);
    aSerial->print(F(", DEFAULT_DRIVE_SPEED_PWM="));
    aSerial->println(DEFAULT_DRIVE_SPEED_PWM);

    aSerial->print(F("DEFAULT_MILLIS_PER_MILLIMETER="));
    aSerial->println(DEFAULT_MILLIS_PER_MILLIMETER);
    aSerial->println();
}

//void PanicWithLed(unsigned int aDelay, uint8_t aCount) {
//    for (uint8_t i = 0; i < aCount; ++i) {
//        digitalWrite(LED_BUILTIN, HIGH);
//        delay(aDelay);
//        digitalWrite(LED_BUILTIN, LOW);
//        delay(aDelay);
//    }
//}

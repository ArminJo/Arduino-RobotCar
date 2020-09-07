/*
 * EncoderMotor.cpp
 *
 *  Functions for controlling a DC-motor which rotary encoder attached.
 *  Works with positive speed and direction.
 *
 *  Contains functions to go a specified distance.
 *  These functions generates ramps for acceleration and deceleration and tries to stop at target distance.
 *  This enables deterministic turns for 2-Wheel Cars.  For 4-Wheel cars it is impossible
 *  to get deterministic turns, therefore I use approximated thumb values.
 *
 *  Tested for Adafruit Motor Shield and plain TB6612 breakout board.
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
#include "EncoderMotor.h"

//#include "RobotCarGui.h"

bool EncoderMotor::MotorValuesHaveChanged; // for printing
volatile bool EncoderMotor::EncoderTickCounterHasChanged;

/*
 * The list version saves 100 bytes and is more flexible, compared with the array version
 */
uint8_t EncoderMotor::sNumberOfMotorControls = 0;
EncoderMotor * EncoderMotor::sMotorControlListStart = NULL;

EncoderMotor::EncoderMotor() : // @suppress("Class members should be properly initialized")
        TB6612DcMotor() {
    /*
     * The list version saves 100 bytes and is more flexible, compared with the array version
     */
    EncoderMotorNumber = EncoderMotor::sNumberOfMotorControls;
    EncoderMotor::sNumberOfMotorControls++;
    NextMotorControl = NULL;
    if (sMotorControlListStart == NULL) {
        // first constructor
        sMotorControlListStart = this;
    } else {
        // put object in control list
        EncoderMotor * tObjectPointer = sMotorControlListStart;
        // search last list element
        while (tObjectPointer->NextMotorControl != NULL) {
            tObjectPointer = tObjectPointer->NextMotorControl;
        }
        //insert current control in last element
        tObjectPointer->NextMotorControl = this;
    }
}

#ifdef USE_TB6612_BREAKOUT_BOARD
void EncoderMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin) {
    TB6612DcMotor::init(aForwardPin, aBackwardPin, aPWMPin);
    /*
     * The list version saves 100 bytes and is more flexible, compared with the array version
     */
    EncoderMotorNumber = EncoderMotor::sNumberOfMotorControls;
    EncoderMotor::sNumberOfMotorControls++;
    NextMotorControl = NULL;
    if (sMotorControlListStart == NULL) {
        // first constructor
        sMotorControlListStart = this;
    } else {
        // put object in control list
        EncoderMotor * tObjectPointer = sMotorControlListStart;
        // search last list element
        while (tObjectPointer->NextMotorControl != NULL) {
            tObjectPointer = tObjectPointer->NextMotorControl;
        }
        //insert current control in last element
        tObjectPointer->NextMotorControl = this;
    }
}
#else
void EncoderMotor::init(uint8_t aMotorNumber) {
    TB6612DcMotor::init(aMotorNumber);  // create with the default frequency 1.6KHz
    // stop motor
    stopMotorAndReset();
    readEeprom();
}
#endif

/*
 * if aDistanceCount < 0 then use DIRECTION_BACKWARD
 * If motor is still running, add aDistanceCount to current target distance
 */
void EncoderMotor::initGoDistanceCount(int aDistanceCount) {
    uint8_t tRequestedDirection = DIRECTION_FORWARD;

    if (aDistanceCount < 0) {
        aDistanceCount = -aDistanceCount;
        tRequestedDirection = DIRECTION_BACKWARD;
    }
    initGoDistanceCount(aDistanceCount, tRequestedDirection);
}

void EncoderMotor::initGoDistanceCount(unsigned int aDistanceCount, uint8_t aRequestedDirection) {
    if (CurrentDirection != aRequestedDirection) {
        CurrentDirection = aRequestedDirection;
        if (State != MOTOR_STATE_STOPPED) {
#ifdef DEBUG
            Serial.print(F("Direction change to"));
            Serial.println(tRequestedDirection);
#endif
            /*
             * Direction change requested but motor still running-> first stop motor
             */
            stopMotor(MOTOR_BRAKE);
            LastTargetDistanceCount = EncoderCount; // Reset LastTargetDistanceCount on direction change
        }
    }

    if (State == MOTOR_STATE_STOPPED) {
        CurrentMaxSpeed = MaxSpeed - SpeedCompensation;
        /*
         * Start the motor and compensate for last distance delta
         * Compensation is only valid if direction does not change.
         */
        // Positive if driven too far, negative if driven too short
        int8_t tLastDelta = (int) EncoderCount - (int) LastTargetDistanceCount;
        if (abs(tLastDelta) <= MAX_DISTANCE_DELTA && (int) aDistanceCount >= tLastDelta) {
            TargetDistanceCount = (int) aDistanceCount - tLastDelta;
        } else {
            TargetDistanceCount = aDistanceCount;
        }
        EncoderCount = 0;
    } else {
        /*
         * Increase the distance to go for running motor
         */
        TargetDistanceCount += aDistanceCount;
        NextChangeMaxTargetCount += aDistanceCount;
    }
    LastTargetDistanceCount = TargetDistanceCount;
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool EncoderMotor::updateMotor() {
    unsigned long tMillis = millis();

    if (State == MOTOR_STATE_STOPPED) {
        if (TargetDistanceCount > 0) {
            //  --> RAMP_UP
            State = MOTOR_STATE_RAMP_UP;
            LastRideEncoderCount = 0;
            /*
             * Start motor
             */
            NextRampChangeMillis = tMillis + RAMP_UP_UPDATE_INTERVAL_MILLIS;
            CurrentSpeed = MinSpeed;
            // not really needed here since output is disabled during ramps
            MotorValuesHaveChanged = true;

            NextChangeMaxTargetCount = TargetDistanceCount / 2;
            // initialize for timeout detection
            EncoderTickLastMillis = tMillis - ENCODER_SENSOR_MASK_MILLIS - 1;

            RampDelta = RAMP_UP_VALUE_DELTA;
            if (RampDelta < 2) {
                RampDelta = 2;
            }
            DebugCount = 0;
            Debug = 0;

            TB6612DcMotor::setSpeed(CurrentSpeed, CurrentDirection);
        }

    } else if (State == MOTOR_STATE_RAMP_UP) {
        /*
         * Increase motor speed
         */
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis += RAMP_UP_UPDATE_INTERVAL_MILLIS;
            CurrentSpeed += RampDelta;
            // Clip value and check for 8 bit overflow
            if (CurrentSpeed > CurrentMaxSpeed || CurrentSpeed <= RampDelta) {
                CurrentSpeed = CurrentMaxSpeed;
            }
            MotorValuesHaveChanged = true;

            /*
             * Transition criteria is:
             * Max Speed reached or more than half of distance is done
             */
            if (CurrentSpeed == CurrentMaxSpeed || EncoderCount >= NextChangeMaxTargetCount) {
                //  --> FULL_SPEED
                State = MOTOR_STATE_FULL_SPEED;

                DistanceCountAfterRampUp = EncoderCount;
                uint8_t tDistanceCountForRampDown = DistanceCountAfterRampUp;
                // guarantee minimal ramp down length
                if (tDistanceCountForRampDown < 3 && TargetDistanceCount > 6) {
                    tDistanceCountForRampDown = 3;
                }
                NextChangeMaxTargetCount = TargetDistanceCount - tDistanceCountForRampDown;
            }
            TB6612DcMotor::setSpeed(CurrentSpeed, CurrentDirection);
        }
    }

    // do not use else if since state can be changed in code before
    if (State == MOTOR_STATE_FULL_SPEED) {
        /*
         * Wait until ramp down count is reached
         */
        if (EncoderCount >= NextChangeMaxTargetCount) {
            NextChangeMaxTargetCount++;
            //  --> RAMP_DOWN
            State = MOTOR_STATE_RAMP_DOWN;
            /*
             * Ramp to reach MinSpeed after 1/2 of remaining distance
             */
            RampDeltaPerDistanceCount = ((CurrentSpeed - StopSpeed) * 2) / ((TargetDistanceCount - EncoderCount)) + 1;
            // brake
            if (CurrentSpeed > RampDeltaPerDistanceCount) {
                CurrentSpeed -= RampDeltaPerDistanceCount;
            } else {
                CurrentSpeed = StopSpeed;
            }
            MotorValuesHaveChanged = true;
            TB6612DcMotor::setSpeed(CurrentSpeed, CurrentDirection);
        }

    } else if (State == MOTOR_STATE_RAMP_DOWN) {
        DebugCount++;

        /*
         * Decrease motor speed depending on distance to target count
         */
        if (EncoderCount >= NextChangeMaxTargetCount) {
            Debug++;
            NextChangeMaxTargetCount++;
            if (CurrentSpeed > RampDeltaPerDistanceCount) {
                CurrentSpeed -= RampDeltaPerDistanceCount;
            } else {
                CurrentSpeed = StopSpeed;
            }
            // safety net for slow speed
            if (CurrentSpeed < StopSpeed) {
                CurrentSpeed = StopSpeed;
            }
            // not really needed here since output is disabled during ramps
            MotorValuesHaveChanged = true;
        }
        /*
         * Check if target count is reached
         */
        if (EncoderCount >= TargetDistanceCount) {
            SpeedAtTargetCountReached = CurrentSpeed;
            stopMotor(MOTOR_BRAKE);
            return false;
        } else {
            TB6612DcMotor::setSpeed(CurrentSpeed, CurrentDirection);
        }
    }

    /*
     * Check for encoder tick timeout
     */
    if (State != MOTOR_STATE_STOPPED && tMillis > (EncoderTickLastMillis + RAMP_DOWN_TIMEOUT_MILLIS)) {
// No encoder tick in the last 500 ms -> stop motor
        SpeedAtTargetCountReached = CurrentSpeed;
        stopMotor(MOTOR_BRAKE);
#ifdef DEBUG
        Serial.println(F("Encoder timeout -> stop motor"));
#endif
        return false;
    }
    return true;
}

/*
 * Computes motor speed compensation value in order to go exactly straight ahead
 * Compensate only at forward direction
 */
void EncoderMotor::synchronizeMotor(EncoderMotor * aOtherMotorControl, uint16_t aCheckInterval) {
    if (CurrentDirection == DIRECTION_BACKWARD) {
        return;
    }
    static long sNextMotorSyncMillis;
    long tMillis = millis();
    if (tMillis >= sNextMotorSyncMillis) {
        sNextMotorSyncMillis += aCheckInterval;
// only synchronize if manually operated or at full speed
        if ((State == MOTOR_STATE_STOPPED && aOtherMotorControl->State == MOTOR_STATE_STOPPED && CurrentSpeed > 0)
                || (State == MOTOR_STATE_FULL_SPEED && aOtherMotorControl->State == MOTOR_STATE_FULL_SPEED)) {

            MotorValuesHaveChanged = false;
            if (EncoderCount >= (aOtherMotorControl->EncoderCount + 2)) {
                EncoderCount = aOtherMotorControl->EncoderCount;
                /*
                 * This motor is too fast, first try to reduce other motors compensation
                 */
                if (aOtherMotorControl->SpeedCompensation >= 2) {
                    aOtherMotorControl->SpeedCompensation -= 2;
                    aOtherMotorControl->CurrentSpeed += 2;
                    aOtherMotorControl->setSpeed(aOtherMotorControl->CurrentSpeed, CurrentDirection);
                    MotorValuesHaveChanged = true;
                    EncoderCount = aOtherMotorControl->EncoderCount;
                } else if (CurrentSpeed > MinSpeed) {
                    /*
                     * else increase this motors compensation
                     */
                    SpeedCompensation += 2;
                    CurrentSpeed -= 2;
                    TB6612DcMotor::setSpeed(CurrentSpeed, CurrentDirection);
                    MotorValuesHaveChanged = true;
                }

            } else if (aOtherMotorControl->EncoderCount >= (EncoderCount + 2)) {
                aOtherMotorControl->EncoderCount = EncoderCount;
                /*
                 * Other motor is too fast, first try to reduce this motors compensation
                 */
                if (SpeedCompensation >= 2) {
                    SpeedCompensation -= 2;
                    CurrentSpeed += 2;
                    TB6612DcMotor::setSpeed(CurrentSpeed, CurrentDirection);
                    MotorValuesHaveChanged = true;
                } else if (aOtherMotorControl->CurrentSpeed > aOtherMotorControl->MinSpeed) {
                    /*
                     * else increase other motors compensation
                     */
                    aOtherMotorControl->SpeedCompensation += 2;
                    aOtherMotorControl->CurrentSpeed -= 2;
                    aOtherMotorControl->setSpeed(aOtherMotorControl->CurrentSpeed, CurrentDirection);
                    MotorValuesHaveChanged = true;
                }
            }

            if (MotorValuesHaveChanged && State == MOTOR_STATE_FULL_SPEED) {
                writeEeprom();
            }
        }
    }
}

/*
 * generates a rising ramp and detects the first movement -> this sets dead band / minimum Speed
 */
void EncoderMotor::calibrate() {
    stopAllMotorsAndReset();
    bool endLoop;
    EncoderMotor * tEncoderMotorControlPointer;
    tEncoderMotorControlPointer = sMotorControlListStart;

    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->MinSpeed = 0;
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }

    /*
     * increase motor speed by 1 until motor moves
     */
    for (uint8_t tSpeed = 20; tSpeed != 0xFF; ++tSpeed) {
        tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
        while (tEncoderMotorControlPointer != NULL) {
            if (tEncoderMotorControlPointer->MinSpeed == 0) {
                tEncoderMotorControlPointer->setSpeed(tSpeed, DIRECTION_FORWARD);
                tEncoderMotorControlPointer->CurrentSpeed = tSpeed;
            }
            tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
        }
        delay(100);
        /*
         * Check if wheel moved
         */
        tEncoderMotorControlPointer = sMotorControlListStart;
        endLoop = true;
        while (tEncoderMotorControlPointer != NULL) {
            /*
             * Store speed after 6 counts (3cm)
             */
            if (tEncoderMotorControlPointer->MinSpeed == 0 && tEncoderMotorControlPointer->EncoderCount > 6) {
                tEncoderMotorControlPointer->MinSpeed = tSpeed;
                tEncoderMotorControlPointer->StopSpeed = tSpeed;
            }
            if (tEncoderMotorControlPointer->MinSpeed == 0) {
                // Do not end loop if one motor still not moving
                endLoop = false;
            }
            tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
        }

        if (endLoop) {
            break;
        }
    }

    /*
     * TODO calibrate StopSpeed separately
     */

    tEncoderMotorControlPointer = sMotorControlListStart;
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->writeEeprom();
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }

    stopAllMotors(false);
}

/******************************************************************************************
 * Special helper functions
 *****************************************************************************************/
/*
 * minSpeed (at which car starts to move) for 8 volt is appr. 35 to 40, for 4.3 volt (USB supply) is appr. 90 to 100
 */
void EncoderMotor::setEepromValuesDefaults() {
    MinSpeed = DEFAULT_MIN_SPEED;
    StopSpeed = DEFAULT_STOP_SPEED;
    MaxSpeed = DEFAULT_MAX_SPEED;
    SpeedCompensation = 0;
}

void EncoderMotor::readEeprom() {
    EepromMotorInfoStruct tEepromMotorInfo;
    eeprom_read_block((void*) &tEepromMotorInfo, (void*) (EncoderMotorNumber * sizeof(EepromMotorInfoStruct)),
            sizeof(EepromMotorInfoStruct));
    /*
     * Plausibility check for values
     */
    setEepromValuesDefaults();
    if (tEepromMotorInfo.MinSpeed < 100) {
        MinSpeed = tEepromMotorInfo.MinSpeed;
    }
    if (tEepromMotorInfo.StopSpeed < 120 && tEepromMotorInfo.StopSpeed > 10) {
        StopSpeed = tEepromMotorInfo.StopSpeed;
    }
    if (tEepromMotorInfo.MaxSpeed > 40) {
        MaxSpeed = tEepromMotorInfo.MaxSpeed;
    }
    if (tEepromMotorInfo.SpeedCompensation < 24) {
        SpeedCompensation = tEepromMotorInfo.SpeedCompensation;
    }
    MotorValuesHaveChanged = true;
}

void EncoderMotor::writeEeprom() {
    EepromMotorInfoStruct tEepromMotorInfo;
    tEepromMotorInfo.MinSpeed = MinSpeed;
    tEepromMotorInfo.StopSpeed = StopSpeed;
    tEepromMotorInfo.MaxSpeed = MaxSpeed;
    tEepromMotorInfo.SpeedCompensation = SpeedCompensation;

    eeprom_write_block((void*) &tEepromMotorInfo, (void*) (EncoderMotorNumber * sizeof(EepromMotorInfoStruct)),
            sizeof(EepromMotorInfoStruct));
}

/*************************
 * Direct motor control
 *************************/

void EncoderMotor::setSpeed(int aRequestedSpeed) {
    TB6612DcMotor::setSpeed(aRequestedSpeed); // output PWM value to motor
}


void EncoderMotor::setSpeed(int aRequestedSpeed, uint8_t aRequestedDirection) {
    TB6612DcMotor::setSpeed(aRequestedSpeed, aRequestedDirection); // output PWM value to motor
}

void EncoderMotor::setCurrentSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    if (aRequestedSpeed == 0) {
        stopMotor();
    } else {
        CurrentDirection = aRequestedDirection;
// avoid underflow
        if (aRequestedSpeed > SpeedCompensation) {
            CurrentSpeed = aRequestedSpeed - SpeedCompensation;
        } else {
            CurrentSpeed = 0;
        }
        TB6612DcMotor::setSpeed(CurrentSpeed, CurrentDirection); // output PWM value to motor
        MotorValuesHaveChanged = true;
    }
}

/*
 * Not used yet
 */
void EncoderMotor::setCurrentSpeedCompensated(int aRequestedSpeed) {
    if (aRequestedSpeed == 0) {
        stopMotor();
    } else {
        if (aRequestedSpeed > 0) {
            CurrentDirection = DIRECTION_FORWARD;
        } else {
            CurrentDirection = DIRECTION_BACKWARD;
            aRequestedSpeed = -aRequestedSpeed;
        }
// avoid underflow
        if (aRequestedSpeed > SpeedCompensation) {
            CurrentSpeed = aRequestedSpeed - SpeedCompensation;
        } else {
            CurrentSpeed = 0;
        }
        TB6612DcMotor::setSpeed(CurrentSpeed, CurrentDirection); // output PWM value to motor
        MotorValuesHaveChanged = true;
    }
}

/*
 * The one and only place where State is set to MOTOR_STATE_STOPPED
 */
void EncoderMotor::stopMotor(uint8_t aStopMode) {
    /*
     * Set state to MOTOR_STATE_STOPPED
     */
    CurrentSpeed = 0;
    State = MOTOR_STATE_STOPPED;
    TargetDistanceCount = 0;
    MotorValuesHaveChanged = true;

    TB6612DcMotor::stop(aStopMode);
}

/*
 * Stop car and reset all control values as speed, distances, debug values to 0x00
 * Leave calibration and compensation values unaffected.
 */
void EncoderMotor::stopMotorAndReset() {
    stopMotor(MOTOR_RELEASE);
    memset(&CurrentSpeed, 0, (((uint8_t *) &Debug) + sizeof(Debug)) - &CurrentSpeed);
// to force display of initial values
    EncoderTickCounterHasChanged = true;
    MotorValuesHaveChanged = true;
}

/***************************************************
 * Encoder functions
 ***************************************************/
void EncoderMotor::handleEncoderInterrupt() {
    long tMillis = millis();
    uint16_t tDeltaMillis = tMillis - EncoderTickLastMillis;
    if (tDeltaMillis <= ENCODER_SENSOR_MASK_MILLIS) {
// signal is ringing
        CurrentVelocity = 99;
    } else {
        EncoderTickLastMillis = tMillis;
        EncoderCount++;
        LastRideEncoderCount++;
        CurrentVelocity = VELOCITY_SCALE_VALUE / tDeltaMillis;
        EncoderTickCounterHasChanged = true;
    }
}

// The code for the interrupt is placed at the calling class since we need a fixed relation between ISR and EncoderMotor
// //ISR for PIN PD2 / RIGHT
// ISR(INT0_vect) {
//    myCar.rightMotorControl.handleEncoderInterrupt();
// }

/******************************************************************************************
 * Static methods
 *****************************************************************************************/
/*
 * Enable both interrupts INT0/D2 or INT1/D3
 */
void EncoderMotor::enableINT0AndINT1Interrupts() {

// interrupt on any logical change
    EICRA |= (1 << ISC00 | 1 << ISC10);
// clear interrupt bit
    EIFR |= (1 << INTF0 | 1 << INTF1);
// enable interrupt on next change
    EIMSK |= (1 << INT0 | 1 << INT1);
}

/*
 * aIntPinNumber can be one of INT0/D2 or INT1/D3 for Atmega328
 */
void EncoderMotor::enableInterruptOnBothEdges(uint8_t aIntPinNumber) {
    if (aIntPinNumber > 1) {
        return;
    }

    if (aIntPinNumber == 0) {
// interrupt on any logical change
        EICRA |= (1 << ISC00);
// clear interrupt bit
        EIFR |= 1 << INTF0;
// enable interrupt on next change
        EIMSK |= 1 << INT0;
    } else {
        EICRA |= (1 << ISC10);
        EIFR |= 1 << INTF1;
        EIMSK |= 1 << INT1;
    }
}

/*****************************************************
 * Static convenience functions affecting all motors.
 * If you have 2 motors, better use CarControl
 *****************************************************/

void EncoderMotor::updateAllMotors() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->updateMotor();
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}

/*
 * Waits until distance is reached
 */
void EncoderMotor::startAndWaitForFullSpeedForAll() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->initGoDistanceCount(INFINITE_DISTANCE_COUNT);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    do {
        EncoderMotor::updateAllMotors();
    } while (!EncoderMotor::allMotorsStarted());
}

void EncoderMotor::initGoDistanceCountForAll(int aDistanceCount) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->initGoDistanceCount(aDistanceCount);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}

/*
 * Waits until distance is reached
 */
void EncoderMotor::goDistanceCountForAll(int aDistanceCount, void (*aLoopCallback)(void)) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->initGoDistanceCount(aDistanceCount);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    waitUntilAllMotorsStopped(aLoopCallback);
}

bool EncoderMotor::allMotorsStarted() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
    bool tAllAreStarted = true;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        if (tEncoderMotorControlPointer->State != MOTOR_STATE_FULL_SPEED) {
            tAllAreStarted = false;
        }
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    return tAllAreStarted;
}

bool EncoderMotor::allMotorsStopped() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
    bool tAllAreStopped = true;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        if (tEncoderMotorControlPointer->State != MOTOR_STATE_STOPPED) {
            tAllAreStopped = false;
        }
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    return tAllAreStopped;
}

/*
 * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_FULL_SPEED to MOTOR_STATE_RAMP_DOWN
 * Use DistanceCountAfterRampUp as ramp down count
 * Busy waits for stop
 */
void EncoderMotor::stopAllMotorsAndWaitUntilStopped() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->NextChangeMaxTargetCount = tEncoderMotorControlPointer->EncoderCount;
        tEncoderMotorControlPointer->TargetDistanceCount = tEncoderMotorControlPointer->EncoderCount
                + tEncoderMotorControlPointer->DistanceCountAfterRampUp;
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }

    /*
     * busy wait for stop
     */
    while (!allMotorsStopped()) {
        updateAllMotors();
    }
}

void EncoderMotor::stopAllMotorsAndReset() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->stopMotorAndReset();
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}

void EncoderMotor::waitUntilAllMotorsStopped(void (*aLoopCallback)(void)) {
    do {
        EncoderMotor::updateAllMotors();
        if (aLoopCallback != NULL) {
            aLoopCallback();
        }
    } while (!EncoderMotor::allMotorsStopped());
}

void EncoderMotor::stopAllMotors(uint8_t aStopMode) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->stopMotor(aStopMode);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}


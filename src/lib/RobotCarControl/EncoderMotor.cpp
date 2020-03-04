/*
 * EncoderMotor.cpp
 *
 *  Functions for controlling a DC-motor which rotary encoder attached.
 *  Generates ramps for acceleration and deceleration and tries to stop at target distance.
 *  This enables deterministic turns for 2-Wheel Cars.
 *  For 4-Wheel cars it is impossible go get deterministic turns, therefore I use approximated thumb values.
 *
 *  Needs Adafruit_MotorShield.cpp or TB6612 breakout board
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
#include <EncoderMotor.h>

//#include "RobotCarGui.h"

bool EncoderMotor::ValuesHaveChanged; // for printing
bool EncoderMotor::EnableValuesPrint = true;
volatile bool EncoderMotor::DistanceTickCounterHasChanged;

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

void EncoderMotor::init(uint8_t aMotorNumber) {
    TB6612DcMotor::init(aMotorNumber);  // create with the default frequency 1.6KHz
    // stop motor
    resetAndShutdown();
    readEeprom();
}

void EncoderMotor::updateMotor() {
    unsigned long tMillis = millis();

    if (State == MOTOR_STATE_STOPPED) {
        if (TargetDistanceCount > 0) {
            //  --> RAMP_UP
            State = MOTOR_STATE_RAMP_UP;
            LastRideDistanceCount = 0;
            /*
             * Start motor
             */
            if (isDirectionForward) {
                run(FORWARD);
            } else {
                run(BACKWARD);
            }
            NextRampChangeMillis = tMillis + RAMP_UP_UPDATE_INTERVAL_MILLIS;
            ActualSpeed = MinSpeed;
            // not really needed here since output is disabled during ramps
            ValuesHaveChanged = true;

            NextChangeMaxTargetCount = TargetDistanceCount / 2;
            // initialize for timeout detection
            DistanceTickLastMillis = tMillis - ENCODER_SENSOR_MASK_MILLIS - 1;

            RampDelta = RAMP_UP_VALUE_DELTA;
            if (RampDelta < 2) {
                RampDelta = 2;
            }
            DebugCount = 0;
            Debug = 0;

            setSpeed(ActualSpeed);
        }

    } else if (State == MOTOR_STATE_RAMP_UP) {
        /*
         * Increase motor speed
         */
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis += RAMP_UP_UPDATE_INTERVAL_MILLIS;
            ActualSpeed += RampDelta;
            // Clip value and check for 8 bit overflow
            if (ActualSpeed > ActualMaxSpeed || ActualSpeed <= RampDelta) {
                ActualSpeed = ActualMaxSpeed;
            }
            ValuesHaveChanged = true;

            /*
             * Transition criteria is:
             * Max Speed reached or more than half of distance is done
             */
            if (ActualSpeed == ActualMaxSpeed || DistanceCount >= NextChangeMaxTargetCount) {
                //  --> FULL_SPEED
                State = MOTOR_STATE_FULL_SPEED;

                DistanceCountAfterRampUp = DistanceCount;
                uint8_t tDistanceCountForRampDown = DistanceCountAfterRampUp;
                // guarantee minimal ramp down length
                if (tDistanceCountForRampDown < 3 && TargetDistanceCount > 6) {
                    tDistanceCountForRampDown = 3;
                }
                NextChangeMaxTargetCount = TargetDistanceCount - tDistanceCountForRampDown;
            }
            setSpeed(ActualSpeed);
        }
    }

    // have to check since skipping MOTOR_STATE_FULL_SPEED must be possible
    if (State == MOTOR_STATE_FULL_SPEED) {
        /*
         * Wait until ramp down count is reached
         */
        if (DistanceCount >= NextChangeMaxTargetCount) {
            NextChangeMaxTargetCount++;
            //  --> RAMP_DOWN
            State = MOTOR_STATE_RAMP_DOWN;
            /*
             * Ramp to reach MinSpeed after 1/2 of remaining distance
             */
            RampDeltaPerDistanceCount = ((ActualSpeed - StopSpeed) * 2) / ((TargetDistanceCount - DistanceCount)) + 1;
            // brake
            if (ActualSpeed > RampDeltaPerDistanceCount) {
                ActualSpeed -= RampDeltaPerDistanceCount;
            } else {
                ActualSpeed = StopSpeed;
            }
            ValuesHaveChanged = true;
            setSpeed(ActualSpeed);
        }

    } else if (State == MOTOR_STATE_RAMP_DOWN) {
        DebugCount++;

        /*
         * Decrease motor speed depending on distance to target count
         */
        if (DistanceCount >= NextChangeMaxTargetCount) {
            Debug++;
            NextChangeMaxTargetCount++;
            if (ActualSpeed > RampDeltaPerDistanceCount) {
                ActualSpeed -= RampDeltaPerDistanceCount;
            } else {
                ActualSpeed = StopSpeed;
            }
            // safety net for slow speed
            if (ActualSpeed < StopSpeed) {
                ActualSpeed = StopSpeed;
            }
            // not really needed here since output is disabled during ramps
            ValuesHaveChanged = true;
        }
        /*
         * Check if target count is reached
         */
        if (DistanceCount >= TargetDistanceCount) {
            SpeedAtTargetCountReached = ActualSpeed;
            shutdownMotor(true);
        } else {
            setSpeed(ActualSpeed);
        }
    }

    /*
     * Check for timeout
     */
    if (State != MOTOR_STATE_STOPPED && tMillis > (DistanceTickLastMillis + RAMP_DOWN_TIMEOUT_MILLIS)) {
        SpeedAtTargetCountReached = ActualSpeed;
        shutdownMotor(true);
    }
}

/*
 * Computes motor speed compensation value in order to go exactly straight ahead
 */
void EncoderMotor::synchronizeMotor(EncoderMotor * aOtherMotorControl, uint16_t aCheckInterval) {
    static long sNextMotorSyncMillis;
    long tMillis = millis();
    if (tMillis >= sNextMotorSyncMillis) {
        sNextMotorSyncMillis += aCheckInterval;
        // only synchronize if manually operated or at full speed
        if ((State == MOTOR_STATE_STOPPED && aOtherMotorControl->State == MOTOR_STATE_STOPPED && ActualSpeed > 0)
                || (State == MOTOR_STATE_FULL_SPEED && aOtherMotorControl->State == MOTOR_STATE_FULL_SPEED)) {

            ValuesHaveChanged = false;
            if (DistanceCount >= (aOtherMotorControl->DistanceCount + 2)) {
                DistanceCount = aOtherMotorControl->DistanceCount;
                /*
                 * This motor is too fast, first try to reduce other motors compensation
                 */
                if (aOtherMotorControl->SpeedCompensation >= 2) {
                    aOtherMotorControl->SpeedCompensation -= 2;
                    aOtherMotorControl->ActualSpeed += 2;
                    aOtherMotorControl->setSpeed(aOtherMotorControl->ActualSpeed);
                    ValuesHaveChanged = true;
                    DistanceCount = aOtherMotorControl->DistanceCount;
                } else if (ActualSpeed > MinSpeed) {
                    /*
                     * else increase this motors compensation
                     */
                    SpeedCompensation += 2;
                    ActualSpeed -= 2;
                    setSpeed(ActualSpeed);
                    ValuesHaveChanged = true;
                }

            } else if (aOtherMotorControl->DistanceCount >= (DistanceCount + 2)) {
                aOtherMotorControl->DistanceCount = DistanceCount;
                /*
                 * Other motor is too fast, first try to reduce this motors compensation
                 */
                if (SpeedCompensation >= 2) {
                    SpeedCompensation -= 2;
                    ActualSpeed += 2;
                    setSpeed(ActualSpeed);
                    ValuesHaveChanged = true;
                } else if (aOtherMotorControl->ActualSpeed > aOtherMotorControl->MinSpeed) {
                    /*
                     * else increase other motors compensation
                     */
                    aOtherMotorControl->SpeedCompensation += 2;
                    aOtherMotorControl->ActualSpeed -= 2;
                    aOtherMotorControl->setSpeed(aOtherMotorControl->ActualSpeed);
                    ValuesHaveChanged = true;
                }
            }

            if (ValuesHaveChanged && State == MOTOR_STATE_FULL_SPEED) {
                writeEeprom();
            }
        }
    }
}

/*
 * generates a rising ramp and detects the first movement -> this sets dead band / minimum Speed
 */
void EncoderMotor::calibrate() {
    resetAndStopAllMotors();
    bool endLoop;
    EncoderMotor * tEncoderMotorControlPointer;
    tEncoderMotorControlPointer = sMotorControlListStart;

    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->MinSpeed = 0;
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    setDirectionForAll(true);

    /*
     * increase motor speed by 1 until motor moves
     */
    for (uint8_t tSpeed = 20; tSpeed != 0xFF; ++tSpeed) {
        tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
        while (tEncoderMotorControlPointer != NULL) {
            if (tEncoderMotorControlPointer->MinSpeed == 0) {
                tEncoderMotorControlPointer->setSpeed(tSpeed);
                tEncoderMotorControlPointer->ActualSpeed = tSpeed;
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
            if (tEncoderMotorControlPointer->MinSpeed == 0 && tEncoderMotorControlPointer->DistanceCount > 6) {
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

    shutdownAllMotors(false);
}

void EncoderMotor::initGoDistanceCount(int aDistanceCount) {

    if (aDistanceCount > 0) {
        isDirectionForward = true;
    } else if (aDistanceCount < 0) {
        aDistanceCount = -aDistanceCount;
        isDirectionForward = false;
    }

    if (State == MOTOR_STATE_STOPPED) {
        ActualMaxSpeed = MaxSpeed - SpeedCompensation;
        /*
         * Start the motor and compensate for last distance delta
         */
        // Positive if driven too far
        int8_t tLastDelta = (int) DistanceCount - (int) LastTargetDistanceCount;
        if (abs(tLastDelta) <= MAX_DISTANCE_DELTA && aDistanceCount >= tLastDelta) {
            TargetDistanceCount = aDistanceCount - tLastDelta;
        } else {
            TargetDistanceCount = aDistanceCount;
        }
        DistanceCount = 0;
    } else {
        /*
         * Increase the distance to go for running motor
         */
        TargetDistanceCount += aDistanceCount;
        NextChangeMaxTargetCount += aDistanceCount;
    }
    LastTargetDistanceCount = TargetDistanceCount;
}

void EncoderMotor::setDirection(bool goForward) {
    isDirectionForward = goForward;
    activate();
}

void EncoderMotor::activate() {
    if (isDirectionForward) {
        run(FORWARD);
    } else {
        run(BACKWARD);
    }
}

void EncoderMotor::shutdownMotor(bool doBrake) {
    setSpeedCompensated(0);
    if (doBrake) {
        run(BRAKE);
    } else {
        run(RELEASE);
    }
}

/*
 * Stop car and reset all control values as speed, distances, debug values to 0x00
 * Leave calibration and compensation values unaffected.
 */
void EncoderMotor::resetAndShutdown() {
    shutdownMotor(false);
    memset(&ActualSpeed, 0, (((uint8_t *) &Debug) + sizeof(Debug)) - &ActualSpeed);
    isDirectionForward = true;
// to force display of initial values
    DistanceTickCounterHasChanged = true;
    ValuesHaveChanged = true;
}

/******************************************************************************************
 * Special helper functions
 *****************************************************************************************/
/*
 * minSpeed (at which car starts to move) for 8 volt is appr. 35 to 40, for 4.3 volt (USB supply) is appr. 90 to 100
 */
void EncoderMotor::readEeprom() {
    EepromMotorInfoStruct tEepromMotorInfo;
    eeprom_read_block((void*) &tEepromMotorInfo, (void*) (EncoderMotorNumber * sizeof(EepromMotorInfoStruct)),
            sizeof(EepromMotorInfoStruct));
    /*
     * Plausibility check for values
     */
    if (tEepromMotorInfo.MinSpeed < 100) {
        MinSpeed = tEepromMotorInfo.MinSpeed;
    } else {
        MinSpeed = DEFAULT_MIN_SPEED;
    }
    if (tEepromMotorInfo.StopSpeed < 120 && tEepromMotorInfo.StopSpeed > 10) {
        StopSpeed = tEepromMotorInfo.StopSpeed;
    } else {
        StopSpeed = DEFAULT_STOP_SPEED;
    }
    if (tEepromMotorInfo.MaxSpeed < 40) {
        MaxSpeed = DEFAULT_MAX_SPEED;
    } else {
        MaxSpeed = tEepromMotorInfo.MaxSpeed;
    }
    if (tEepromMotorInfo.SpeedCompensation < 24) {
        SpeedCompensation = tEepromMotorInfo.SpeedCompensation;
    } else {
        SpeedCompensation = 0;
    }
    ValuesHaveChanged = true;
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

void EncoderMotor::handleEncoderInterrupt() {
    long tMillis = millis();
    uint16_t tDeltaMillis = tMillis - DistanceTickLastMillis;
    if (tDeltaMillis <= ENCODER_SENSOR_MASK_MILLIS) {
        // signal is ringing
        ActualVelocity = 99;
    } else {
        DistanceTickLastMillis = tMillis;
        DistanceCount++;
        LastRideDistanceCount++;
        ActualVelocity = VELOCITY_SCALE_VALUE / tDeltaMillis;
        DistanceTickCounterHasChanged = true;
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

void EncoderMotor::updateAllMotors() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->updateMotor();
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}

void EncoderMotor::setDirectionForAll(bool goForward) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->setDirection(goForward);
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
        tEncoderMotorControlPointer->initGoDistanceCount(3000);
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

/*
 * The one and only place where State is set to MOTOR_STATE_STOPPED
 */
void EncoderMotor::setSpeedCompensated(uint8_t aRequestedSpeed) {
    if (aRequestedSpeed == 0) {
        /*
         * Set state to MOTOR_STATE_STOPPED and update LastRideDistanceCount
         */
        ActualSpeed = 0;
        State = MOTOR_STATE_STOPPED;
        TargetDistanceCount = 0;
    } else if (aRequestedSpeed > SpeedCompensation) {
        ActualSpeed = aRequestedSpeed - SpeedCompensation;
    } else {
        ActualSpeed = 0;
    }
    setSpeed(ActualSpeed); // output value to motor
    ValuesHaveChanged = true;
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
        tEncoderMotorControlPointer->NextChangeMaxTargetCount = tEncoderMotorControlPointer->DistanceCount;
        tEncoderMotorControlPointer->TargetDistanceCount = tEncoderMotorControlPointer->DistanceCount
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

void EncoderMotor::resetAndStopAllMotors() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->resetAndShutdown();
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

void EncoderMotor::shutdownAllMotors(bool doBrake) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->shutdownMotor(doBrake);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}


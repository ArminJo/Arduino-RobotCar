/*
 * TB6612Motor.cpp
 *
 * Low level Motor control for Adafruit_MotorShield OR breakout board with TB6612FNG / Driver IC for Dual DC motor
 *
 * Motor control has 2 parameters:
 * 1. Speed / PWM which is ignored for BRAKE or RELEASE. This library also accepts signed speed (including the direction as sign).
 * 2. Direction / MotorDriverMode. Can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 *
 *  Created on: 12.05.2019
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

#include "TB6612DcMotor.h"

#if ! defined(USE_TB6612_BREAKOUT_BOARD)
// Create the motor shield object with the default I2C address
Adafruit_MotorShield sAdafruitMotorShield = Adafruit_MotorShield();
#endif

TB6612DcMotor::TB6612DcMotor() { // @suppress("Class members should be properly initialized")
}

#ifdef USE_TB6612_BREAKOUT_BOARD
void TB6612DcMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin) {
    ForwardPin = aForwardPin;
    BackwardPin = aBackwardPin;
    PWMPin = aPWMPin;
    StopMode = MOTOR_RELEASE;

    pinMode(aForwardPin, OUTPUT);
    pinMode(aBackwardPin, OUTPUT);
    pinMode(aPWMPin, OUTPUT);
}

#else
/*
 * aMotorNumber from 0 to 3
 */
void TB6612DcMotor::init(uint8_t aMotorNumber) {

    Adafruit_MotorShield_DcMotor = sAdafruitMotorShield.getMotor(aMotorNumber + 1);
    sAdafruitMotorShield.begin();
}#endif

/*
 *  @brief  Control the DC motor driver direction and stop mode
 *  @param  aMotorDriverMode The mode can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 */
void TB6612DcMotor::setMotorDriverMode(uint8_t aMotorDriverMode) {
#ifdef USE_TB6612_BREAKOUT_BOARD
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
#else
    // until here DIRECTION_FORWARD is 0 back is 1, Adafruit library starts with 1
    Adafruit_MotorShield_DcMotor->run(aMotorDriverMode + CONVERSION_FOR_ADAFRUIT_API);
#endif

}

/*
 *  @brief  Control the DC Motor speed/throttle
 *  @param  speed The 8-bit PWM value, 0 is off, 255 is on forward -255 is on backward
 *  First set driver mode, then set PWM
 */
void TB6612DcMotor::setSpeed(uint8_t aSpeedRequested, uint8_t aDirection) {
    if (aSpeedRequested == 0) {
        setMotorDriverMode(StopMode);
    }
    setMotorDriverMode(aDirection);
#ifdef USE_TB6612_BREAKOUT_BOARD
    analogWrite(PWMPin, aSpeedRequested);
#else
    Adafruit_MotorShield_DcMotor->setSpeed(aSpeedRequested);
#endif
}

void TB6612DcMotor::setSpeed(int aSpeedRequested) {
    if (aSpeedRequested == 0) {
        setMotorDriverMode(StopMode);
    } else if (aSpeedRequested < 0) {
        aSpeedRequested = -aSpeedRequested;
        setMotorDriverMode(DIRECTION_BACKWARD);
    } else {
        setMotorDriverMode(DIRECTION_FORWARD);
    }
#ifdef USE_TB6612_BREAKOUT_BOARD
    analogWrite(PWMPin, aSpeedRequested);
#else
    Adafruit_MotorShield_DcMotor->setSpeed(aSpeedRequested);
#endif
}

/*
 * First set PWM to 0 then disable driver
 * aStopMode can be: MOTOR_BRAKE or MOTOR_RELEASE
 */
void TB6612DcMotor::stop(uint8_t aStopMode) {
#ifdef USE_TB6612_BREAKOUT_BOARD
    analogWrite(PWMPin, 0);
#else
    Adafruit_MotorShield_DcMotor->setSpeed(0);
#endif
    setMotorDriverMode(CheckStopMODE(aStopMode));
}

void TB6612DcMotor::setStopMode(uint8_t aStopMode) {
    StopMode = CheckStopMODE(aStopMode);
}

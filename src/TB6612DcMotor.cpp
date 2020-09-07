/*
 * TB6612Motor.cpp
 *
 * Low level motor control for Adafruit_MotorShield OR breakout board with TB6612FNG / Driver IC for dual DC motor
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
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
void TB6612DcMotor::I2CWriteByte(uint8_t aAddress, uint8_t aData) {
    Wire.beginTransmission(0x60);
    Wire.write(aAddress);
    Wire.write(aData);
    Wire.endTransmission();
}

void TB6612DcMotor::I2CSetPWM(uint8_t aPin, uint16_t aOn, uint16_t aOff) {
    Wire.beginTransmission(0x60);
    Wire.write((PCA9685_FIRST_PWM_REGISTER) + 4 * aPin);
    Wire.write(aOn);
    Wire.write(aOn >> 8);
    Wire.write(aOff);
    Wire.write(aOff >> 8);
    Wire.endTransmission();
}

void TB6612DcMotor::I2CSetPin(uint8_t aPin, bool aSetToOn) {
    if (aSetToOn) {
        I2CSetPWM(aPin, 4096, 0);
    } else {
        I2CSetPWM(aPin, 0, 0);
    }
}

#  else
// Create the motor shield object with the default I2C address
Adafruit_MotorShield sAdafruitMotorShield = Adafruit_MotorShield();
#  endif // USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD

/*
 * aMotorNumber from 0 to 1
 * Currently motors 2 and 3 are not required/supported
 */
void TB6612DcMotor::init(uint8_t aMotorNumber) {
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    if (aMotorNumber == 0) {
        PWMPin = 8;
        BackwardPin = 9;
        ForwardPin = 10;
    } else {
        PWMPin = 13;
        BackwardPin = 12;
        ForwardPin = 11;
    }

    Wire.begin();
#if defined (ARDUINO_ARCH_AVR) // Other platforms do not have this new function
    Wire.setWireTimeout(5000); // Sets timeout to 5 ms. default is 25 ms.
#endif
    // Reset PCA9685
    Wire.beginTransmission(PCA9685_GENERAL_CALL_ADDRESS);
    Wire.write(PCA9685_SOFTWARE_RESET);
    Wire.endTransmission();
    // Set expander to 1600 HZ
    I2CWriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_MODE_1_SLEEP)); // go to sleep
    I2CWriteByte(PCA9685_PRESCALE_REGISTER, PCA9685_PRESCALER_FOR_1600_HZ); // set the prescaler
    delay(2); // > 500 us before the restart bit according to datasheet
    I2CWriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_MODE_1_RESTART) | _BV(PCA9685_MODE_1_AUTOINCREMENT)); // reset sleep and enable auto increment

#  else
    Adafruit_MotorShield_DcMotor = sAdafruitMotorShield.getMotor(aMotorNumber + 1);
    sAdafruitMotorShield.begin();
#  endif

}
#endif // USE_TB6612_BREAKOUT_BOARD

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
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    switch (aMotorDriverMode) {
    case DIRECTION_FORWARD:
        I2CSetPin(BackwardPin, LOW); // take low first to avoid 'break'
        I2CSetPin(ForwardPin, HIGH);
        break;
    case DIRECTION_BACKWARD:
        I2CSetPin(ForwardPin, LOW); // take low first to avoid 'break'
        I2CSetPin(BackwardPin, HIGH);
        break;
    case MOTOR_BRAKE:
        I2CSetPin(ForwardPin, HIGH);
        I2CSetPin(BackwardPin, HIGH);
        break;
    case MOTOR_RELEASE:
        I2CSetPin(ForwardPin, LOW);
        I2CSetPin(BackwardPin, LOW);
        break;
    }
#  else
    Adafruit_MotorShield_DcMotor->run(aMotorDriverMode + CONVERSION_FOR_ADAFRUIT_API);
#  endif
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
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    I2CSetPWM(PWMPin, 0, 16 * aSpeedRequested);
#  else
    Adafruit_MotorShield_DcMotor->setSpeed(aSpeedRequested);
#  endif
#endif
}

void TB6612DcMotor::setSpeed(int aSpeedRequested) {
    if (aSpeedRequested < 0) {
        aSpeedRequested = -aSpeedRequested;
        setSpeed(aSpeedRequested, DIRECTION_BACKWARD);
    } else {
        setSpeed(aSpeedRequested, DIRECTION_FORWARD);
    }
}

/*
 * First set PWM to 0 then disable driver
 * aStopMode can be: MOTOR_BRAKE or MOTOR_RELEASE
 */
void TB6612DcMotor::stop(uint8_t aStopMode) {
#ifdef USE_TB6612_BREAKOUT_BOARD
    analogWrite(PWMPin, 0);
#else
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    setSpeed(0, DIRECTION_FORWARD);
#  else
    Adafruit_MotorShield_DcMotor->setSpeed(0);
#  endif
#endif
    setMotorDriverMode(CheckStopMODE(aStopMode));
}

void TB6612DcMotor::setStopMode(uint8_t aStopMode) {
    StopMode = CheckStopMODE(aStopMode);
}

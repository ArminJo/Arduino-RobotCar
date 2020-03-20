/*
 * TB6612Motor.cpp
 *
 *  Created on: 12.05.2019
 *      Author: Armin
 *
 * Motor control has 2 technical dimensions
 * 1. Motor driver control. Can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 * 2. Speed / PWM which is ignored for BRAKE or RELEASE
 */

#include <Arduino.h>

#include "TB6612DcMotor.h"

#ifdef USE_TB6612_BREAKOUT_BOARD
#include "RobotCar.h" // for motor pin definitions
#else
// Create the motor shield object with the default I2C address
Adafruit_MotorShield sAdafruitMotorShield = Adafruit_MotorShield();
#endif

TB6612DcMotor::TB6612DcMotor() { // @suppress("Class members should be properly initialized")
}

#ifdef USE_TB6612_BREAKOUT_BOARD
TB6612DcMotor::TB6612DcMotor(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin) {
    ForwardPin = aForwardPin;
    BackwardPin = aBackwardPin;
    PWMPin = aPWMPin;
    StopMode = MOTOR_RELEASE;

    pinMode(aForwardPin, OUTPUT);
    pinMode(aBackwardPin, OUTPUT);
    pinMode(aPWMPin, OUTPUT);
}
#endif

/*
 * aMotorNumber from 0 to 3
 */
void TB6612DcMotor::init(uint8_t aMotorNumber) {
#ifdef USE_TB6612_BREAKOUT_BOARD
    if (aMotorNumber == 0) {
        ForwardPin = PIN_MOTOR_0_FORWARD;
        BackwardPin = PIN_MOTOR_0_BACKWARD;
        PWMPin = PIN_MOTOR_0_PWM;
    } else {
        ForwardPin = PIN_MOTOR_1_FORWARD;
        BackwardPin = PIN_MOTOR_1_BACKWARD;
        PWMPin = PIN_MOTOR_1_PWM;
    }

    StopMode = MOTOR_RELEASE;

    pinMode(ForwardPin, OUTPUT);
    pinMode(BackwardPin, OUTPUT);
    pinMode(PWMPin, OUTPUT);
#else
    Adafruit_MotorShield_DcMotor = sAdafruitMotorShield.getMotor(aMotorNumber + 1);
    sAdafruitMotorShield.begin();
#endif
}

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
 */
void TB6612DcMotor::stop(uint8_t aStopMode) {
#ifdef USE_TB6612_BREAKOUT_BOARD
    analogWrite(PWMPin, 0);
#else
    Adafruit_MotorShield_DcMotor->setSpeed(0);
#endif
    setMotorDriverMode(CheckStopMODE(StopMode));
}

void TB6612DcMotor::setStopMode(uint8_t aStopMode) {
    StopMode = CheckStopMODE(StopMode);
}

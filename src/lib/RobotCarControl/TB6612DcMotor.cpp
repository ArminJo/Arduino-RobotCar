/*
 * TB6612Motor.cpp
 *
 *  Created on: 12.05.2019
 *      Author: Armin
 */

#include <Arduino.h>

#include <TB6612DcMotor.h>

#ifndef USE_TB6612_BREAKOUT_BOARD
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
        ForwardPin = MOTOR_0_FORWARD_PIN;
        BackwardPin = MOTOR_0_BACKWARD_PIN;
        PWMPin = MOTOR_0_PWM_PIN;
    } else {
        ForwardPin = MOTOR_1_FORWARD_PIN;
        BackwardPin = MOTOR_1_BACKWARD_PIN;
        PWMPin = MOTOR_1_PWM_PIN;
    }
    pinMode(ForwardPin, OUTPUT);
    pinMode(BackwardPin, OUTPUT);
    pinMode(PWMPin, OUTPUT);
#else
    Adafruit_MotorShield_DcMotor = sAdafruitMotorShield.getMotor(aMotorNumber + 1);
    sAdafruitMotorShield.begin();
#endif
}

/*
 *  @brief  Control the DC Motor direction and action
 *  @param  cmd The action to perform, can be FORWARD, BACKWARD or RELEASE
 */
void TB6612DcMotor::run(uint8_t cmd) {
#ifdef USE_TB6612_BREAKOUT_BOARD
    switch (cmd) {
        case FORWARD:
        digitalWrite(BackwardPin, LOW); // take low first to avoid 'break'
        digitalWrite(ForwardPin, HIGH);
        break;
        case BACKWARD:
        digitalWrite(ForwardPin, LOW);// take low first to avoid 'break'
        digitalWrite(BackwardPin, HIGH);
        break;
        case RELEASE:
        digitalWrite(ForwardPin, LOW);
        digitalWrite(BackwardPin, LOW);
        break;
        case BRAKE:
        digitalWrite(ForwardPin, HIGH);
        digitalWrite(BackwardPin, HIGH);
        break;
    }
#else
    Adafruit_MotorShield_DcMotor->run(cmd);
#endif

}

void TB6612DcMotor::forward(uint8_t aSpeed) {
#ifdef USE_TB6612_BREAKOUT_BOARD
    digitalWrite(ForwardPin, HIGH);
    digitalWrite(BackwardPin, LOW);
    analogWrite(PWMPin, aSpeed);
#else
    Adafruit_MotorShield_DcMotor->run(FORWARD);
    Adafruit_MotorShield_DcMotor->setSpeed(aSpeed);
#endif
}

void TB6612DcMotor::backward(uint8_t aSpeed) {
#ifdef USE_TB6612_BREAKOUT_BOARD
    digitalWrite(ForwardPin, LOW);
    digitalWrite(BackwardPin, HIGH);
    analogWrite(PWMPin, aSpeed);
#else
    Adafruit_MotorShield_DcMotor->run(BACKWARD);
    Adafruit_MotorShield_DcMotor->setSpeed(aSpeed);
#endif
}

void TB6612DcMotor::brake() {
#ifdef USE_TB6612_BREAKOUT_BOARD
    digitalWrite(ForwardPin, HIGH);
    digitalWrite(BackwardPin, HIGH);
    analogWrite(PWMPin, 0);
#else
    Adafruit_MotorShield_DcMotor->run(BRAKE);
    Adafruit_MotorShield_DcMotor->setSpeed(0);
#endif
}

void TB6612DcMotor::release() {
#ifdef USE_TB6612_BREAKOUT_BOARD
    digitalWrite(ForwardPin, HIGH);
    digitalWrite(BackwardPin, HIGH);
    analogWrite(PWMPin, 0);
#else
    Adafruit_MotorShield_DcMotor->run(RELEASE);
    Adafruit_MotorShield_DcMotor->setSpeed(0);
#endif
}

/*
 *  @brief  Control the DC Motor speed/throttle
 *  @param  speed The 8-bit PWM value, 0 is off, 255 is on
 */
void TB6612DcMotor::setSpeed(uint8_t aSpeed) {
#ifdef USE_TB6612_BREAKOUT_BOARD
    analogWrite(PWMPin, aSpeed);
#else
    Adafruit_MotorShield_DcMotor->setSpeed(aSpeed);
#endif
}

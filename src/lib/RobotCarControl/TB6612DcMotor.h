/*
 * TB6612Motor.h
 *
 *  Created on: 12.05.2019
 *      Author: Armin
 */

#ifndef TB6612DCMOTOR_H_
#define TB6612DCMOTOR_H_

#include <stdint.h>

// uncomment this for building version without adafruit motor shield
//#define USE_TB6612_BREAKOUT_BOARD
#ifdef USE_TB6612_BREAKOUT_BOARD
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4
#else
#include <Adafruit_MotorShield.h>
#endif

/*
 * Pins 9 + 10 are used for Servo library
 * 3 is used for encoder input
 */

#define MOTOR_0_FORWARD_PIN 4
#define MOTOR_0_BACKWARD_PIN 7
#define MOTOR_0_PWM_PIN 5 // PWM capable

#define MOTOR_1_FORWARD_PIN 8
#define MOTOR_1_BACKWARD_PIN 12
#define MOTOR_1_PWM_PIN 6 // PWM capable

class TB6612DcMotor {
public:
#ifdef USE_TB6612_BREAKOUT_BOARD
    TB6612DcMotor(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin);
#endif
    TB6612DcMotor();

    void init(uint8_t aMotorNumber);
    void run(uint8_t cmd);

    void forward(uint8_t aSpeed);
    void backward(uint8_t aSpeed);
    void brake();
    void release();
    void setSpeed(uint8_t aSpeed);

#ifndef USE_TB6612_BREAKOUT_BOARD
    Adafruit_DCMotor * Adafruit_MotorShield_DcMotor;
#else
    uint8_t PWMPin;     // so set speed
    uint8_t ForwardPin; // if high, motor runs forward
    uint8_t BackwardPin;
#endif

};

#endif /* TB6612DCMOTOR_H_ */

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
#if ! defined(USE_TB6612_BREAKOUT_BOARD)
#include <Adafruit_MotorShield.h>
#endif
// Motor directions and stop modes
#define DIRECTION_FORWARD  0
#define DIRECTION_BACKWARD 1
#define changeDIRECTION(aDirection) (aDirection ^ DIRECTION_BACKWARD)
#define MOTOR_BRAKE 2
#define MOTOR_RELEASE 3
#define STOP_MODE_AND_MASK 0x03
#define STOP_MODE_OR_MASK 0x02
#define CheckStopMODE(aStopMode) ((aStopMode & STOP_MODE_AND_MASK) | STOP_MODE_OR_MASK)
#define CONVERSION_FOR_ADAFRUIT_API 1

/*
 * Motor control has 2 technical dimensions
 * 1. Motor driver control. Can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 * 2. Speed / PWM which is ignored for BRAKE or RELEASE
 */
class TB6612DcMotor {
public:
#ifdef USE_TB6612_BREAKOUT_BOARD
    TB6612DcMotor(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin);
#endif
    TB6612DcMotor();

    void init(uint8_t aMotorNumber);
    void setMotorDriverMode(uint8_t cmd);

    void setStopMode(uint8_t aStopMode); // mode for Speed==0, MOTOR_BRAKE or MOTOR_RELEASE
    void setSpeed(int aSpeedRequested);
    void setSpeed(uint8_t aSpeedRequested, uint8_t aDirection);
    void stop(uint8_t aStopMode); // sets speed to 0 and MOTOR_BRAKE or MOTOR_RELEASE

#ifndef USE_TB6612_BREAKOUT_BOARD
    Adafruit_DCMotor * Adafruit_MotorShield_DcMotor;
#else
    uint8_t PWMPin;     // so set speed
    uint8_t ForwardPin; // if high, motor runs forward
    uint8_t BackwardPin;
#endif
    uint8_t StopMode;
};

#endif /* TB6612DCMOTOR_H_ */

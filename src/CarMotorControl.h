/*
 * CarMotorControl.h
 *
 *  Motor control for a car with 2 encoder motors
 *
 *  Created on: 16.09.2016
 *      Author: Armin
 */

#ifndef CARMOTORCONTROL_H_
#define CARMOTORCONTROL_H_

#include "EncoderMotor.h"
#include <stdint.h>

/*
 * Some factors depending on wheel diameter and encoder resolution
 */
#define FACTOR_CENTIMETER_TO_COUNT 2
// here we have 40 ticks per turn and circumference is 21.5 cm
#define FACTOR_DEGREE_TO_COUNT_2WD_CAR 0.4277777
#define FACTOR_DEGREE_TO_COUNT_4WD_CAR 0.8
#define INFINITE_DISTANCE_CM (INFINITE_DISTANCE_COUNT / FACTOR_CENTIMETER_TO_COUNT)

// turn directions
#define TURN_FORWARD DIRECTION_FORWARD
#define TURN_BACKWARD DIRECTION_BACKWARD
#define TURN_IN_PLACE 2

class CarMotorControl {
public:

    CarMotorControl();
//    virtual ~CarMotorControl();

    void init(uint8_t aPinFor2WDDetection);
    void setDefaultsForFixedDistanceDriving();

    /*
     * Functions for moving a fixed distance
     */
    void updateMotors();

    void initGoDistanceCentimeter(int aDistanceCentimeter); // only setup values
    void goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void) = NULL); // Blocking function, uses waitUntilCarStopped
    void waitUntilCarStopped(void (*aLoopCallback)(void));

    /*
     * Functions for rotation
     */
    void initRotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed = true);
    void rotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection = TURN_IN_PLACE, bool aUseSlowSpeed = true);
    void rotateCar(int16_t aRotationDegrees, void (*aLoopCallback)(void), uint8_t aTurnDirection = TURN_IN_PLACE,
            bool aUseSlowSpeed = true);

    /*
     * Start/Stop functions for infinite distance
     */
    void startCarAndWaitForFullSpeed(uint8_t aDriveDirection = DIRECTION_FORWARD);
    void stopCarAndWaitForIt(); // uses waitUntilCarStopped()
    void waitUntilCarStopped();

    /*
     * Check motor state functions
     */
    bool isStopped();
    bool isStarted();
    bool isState(uint8_t aState);
    bool needsFastUpdates();

    /*
     * Functions, which directly call EncoderMotor functions for both motors
     */
    void setSpeed(int aRequestedSpeed);
    void setSpeedCompensated(int aRequestedSpeed);
    void stopMotors(uint8_t aStopMode = MOTOR_RELEASE);
    void stopMotorsAndReset();

    bool is2WDCar;
};

extern EncoderMotor rightEncoderMotor;
extern EncoderMotor leftEncoderMotor;

#endif /* CARMOTORCONTROL_H_ */

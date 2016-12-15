/*
 * CarControl.h
 *
 *  Created on: 16.09.2016
 *      Author: Armin
 */

#ifndef SRC_CARCONTROL_H_
#define SRC_CARCONTROL_H_

#include <stdint.h>
//#include <Adafruit_MotorShield.h>
#include "EncoderMotorControl.h"

/*
 * Some factors depending on wheel diameter and encoder resolution
 */
#define FACTOR_CENTIMETER_TO_COUNT 2
// here we have 40 ticks per turn and circumference is 21.5 cm
#define FACTOR_DEGREE_TO_COUNT_2WD_CAR 0.4277777
#define FACTOR_DEGREE_TO_COUNT_4WD_CAR 0.8

#define TURN_FORWARD 1
#define TURN_BACKWARD 2
#define TURN_IN_PLACE 3

/*
 * Car Control
 */

class CarControl {
public:

    CarControl();
    virtual ~CarControl();

    void init(uint8_t aPinFor2WDDetection);

    /*
     * Functions, which directly call EncoderMotorControl functions for both motors
     */
    void setSpeedCompensated(uint8_t aSpeed);
    //This stops motors
    void setDirection(bool goForward);
    void updateMotors();
    void activateMotors();
    void shutdownMotors(bool doBrake);
    void resetAndShutdownMotors();

    /*
     * State functions
     */
    bool isStopped();
    //
    bool isStarted();
    //
    bool isState(uint8_t aState);

    /*
     * Moving with wait
     */
    void initGoDistanceCentimeter(int aDistanceCentimeter);
    void goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void));

    /*
     * Rotation with wait
     */
    void initRotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection = TURN_IN_PLACE, bool aUseSlowSpeed = true);
    void rotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection = TURN_IN_PLACE, bool aUseSlowSpeed = true);
    void rotateCar(int16_t aRotationDegrees, void (*aLoopCallback)(void), uint8_t aTurnDirection = TURN_IN_PLACE,
            bool aUseSlowSpeed = true);

    /*
     * Start/Stop with wait
     */
    void startAndWaitForFullSpeed();
    void stopCar();
    void waitUntilCarStopped();
    void waitUntilCarStopped(void (*aLoopCallback)(void));

    EncoderMotorControl rightMotorControl;
    EncoderMotorControl leftMotorControl;
    // true if forward
    bool isDirectionForward;
    //
    bool is2WDCar;
};

#endif /* SRC_CARCONTROL_H_ */

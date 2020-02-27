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

#include <EncoderMotor.h>
#include <stdint.h>

/*
 * Some factors depending on wheel diameter and encoder resolution
 */
#define FACTOR_CENTIMETER_TO_COUNT 2
// here we have 40 ticks per turn and circumference is 21.5 cm
#define FACTOR_DEGREE_TO_COUNT_2WD_CAR 0.4277777
#define FACTOR_DEGREE_TO_COUNT_4WD_CAR 0.8

#define TURN_FORWARD 0
#define TURN_BACKWARD 1
#define TURN_IN_PLACE 2

class CarMotorControl {
public:

    CarMotorControl();
//    virtual ~CarMotorControl();

    void init(uint8_t aPinFor2WDDetection);

    /*
     * Functions, which directly call EncoderMotor functions for both motors
     */
    void setSpeedCompensated(uint8_t aSpeed);
    //This stops motors
    void setDirection(bool goForward);
    void updateMotors();
    bool needsFastUpdates();
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
    void initRotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed = true);
    void rotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed = true);
    void rotateCar(int16_t aRotationDegrees, void (*aLoopCallback)(void), uint8_t aTurnDirection = TURN_IN_PLACE,
            bool aUseSlowSpeed = true);

    /*
     * Start/Stop with wait
     */
    void startAndWaitForFullSpeed();
    void stopCar();
    void waitUntilCarStopped();
    void waitUntilCarStopped(void (*aLoopCallback)(void));


    // true if forward
    bool isDirectionForward;
    //
    bool is2WDCar;
};

extern EncoderMotor rightEncoderMotor;
extern EncoderMotor leftEncoderMotor;

#endif /* CARMOTORCONTROL_H_ */

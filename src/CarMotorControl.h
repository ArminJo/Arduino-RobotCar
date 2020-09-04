/*
 * CarMotorControl.h
 *
 *  Motor control for a car with 2 encoder motors
 *
 *  Created on: 16.09.2016
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

#ifdef USE_TB6612_BREAKOUT_BOARD
    void init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin, uint8_t aLeftMotorForwardPin,
            uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin, uint8_t aPinFor2WDDetection);
#else
    void init(uint8_t aPinFor2WDDetection);
#endif
    void setDefaultsForFixedDistanceDriving();

    /*
     * Functions for moving a fixed distance
     */
    bool updateMotors();

    void initGoDistanceCentimeter(int aDistanceCentimeter); // only setup values
    void initGoDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection);
    void goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void) = NULL); // Blocking function, uses waitUntilCarStopped
    void goDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection, void (*aLoopCallback)(void) = NULL); // Blocking function, uses waitUntilCarStopped
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
    void setCurrentSpeedCompensated(int aRequestedSpeed);
    void setCurrentSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection);
    void stopMotors(uint8_t aStopMode = MOTOR_RELEASE);
    void stopMotorsAndReset();

    bool is2WDCar;
};

extern EncoderMotor rightEncoderMotor;
extern EncoderMotor leftEncoderMotor;

#endif /* CARMOTORCONTROL_H_ */

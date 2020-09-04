/*
 * TB6612Motor.h
 *
 *  Created on: 12.05.2019
 *  Copyright (C) 2019-2020  Armin Joachimsmeyer
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

#ifndef TB6612DCMOTOR_H_
#define TB6612DCMOTOR_H_

#include <stdint.h>

// Comment this out for building version for breakout board with TB6612FNG / Driver IC for Dual DC motor
//#define USE_TB6612_BREAKOUT_BOARD
#if ! defined(USE_TB6612_BREAKOUT_BOARD)
#include <Adafruit_MotorShield.h>
#endif
// Motor directions and stop modes. Are used for parameter aMotorDriverMode and sequence is determined by the Adafruit library API.
#define DIRECTION_FORWARD  0
#define DIRECTION_BACKWARD 1
#define oppositeDIRECTION(aDirection) (aDirection ^ DIRECTION_BACKWARD)
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
    TB6612DcMotor();

#ifdef USE_TB6612_BREAKOUT_BOARD
    void init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin);
#else
    void init(uint8_t aMotorNumber);
#endif

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
    uint8_t StopMode; // used for speed == 0
};

#endif /* TB6612DCMOTOR_H_ */

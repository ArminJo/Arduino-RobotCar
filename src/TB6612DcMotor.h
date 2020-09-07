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

#if ! defined(USE_TB6612_BREAKOUT_BOARD)
#  if ! defined(USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
/*
 * Saves 694 bytes program memory
 */
#define USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
#  endif
#include <Wire.h>

// Comment this out for building version for breakout board with TB6612FNG / Driver IC for Dual DC motor
//#define USE_TB6612_BREAKOUT_BOARD
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
// some PCA9685 specific constants
#define PCA9685_GENERAL_CALL_ADDRESS 0x00
#define PCA9685_SOFTWARE_RESET      6
#define PCA9685_DEFAULT_ADDRESS     0x40
#define PCA9685_MAX_CHANNELS        16 // 16 PWM channels on each PCA9685 expansion module
#define PCA9685_MODE1_REGISTER      0x0
#define PCA9685_MODE_1_RESTART        7
#define PCA9685_MODE_1_AUTOINCREMENT  5
#define PCA9685_MODE_1_SLEEP          4
#define PCA9685_FIRST_PWM_REGISTER  0x06
#define PCA9685_PRESCALE_REGISTER   0xFE

#define PCA9685_PRESCALER_FOR_1600_HZ ((25000000L /(4096L * 1600))-1) // = 3 at 1600 Hz

#  else
#include <Adafruit_MotorShield.h>
#  endif
#endif // ! defined(USE_TB6612_BREAKOUT_BOARD)

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
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    void I2CWriteByte(uint8_t aAddress, uint8_t aData);
    void I2CSetPWM(uint8_t aPin, uint16_t aOn, uint16_t aOff);
    void I2CSetPin(uint8_t aPin, bool aSetToOn);

#  else
    Adafruit_DCMotor * Adafruit_MotorShield_DcMotor;
#  endif
#endif

    void setMotorDriverMode(uint8_t cmd);

    void setStopMode(uint8_t aStopMode); // mode for Speed==0, MOTOR_BRAKE or MOTOR_RELEASE
    void setSpeed(int aSpeedRequested);
    void setSpeed(uint8_t aSpeedRequested, uint8_t aDirection);
    void stop(uint8_t aStopMode); // sets speed to 0 and MOTOR_BRAKE or MOTOR_RELEASE

#if defined(USE_TB6612_BREAKOUT_BOARD) || defined(USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    uint8_t PWMPin;     // so set speed
    uint8_t ForwardPin; // if high, motor runs forward
    uint8_t BackwardPin;
#endif
    uint8_t StopMode; // used for speed == 0
};

#endif /* TB6612DCMOTOR_H_ */

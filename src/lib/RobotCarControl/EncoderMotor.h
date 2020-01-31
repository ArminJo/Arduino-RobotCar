/*
 * EncoderMotor.h
 *
 *  Created on: 16.09.2016
 *      Author: Armin
 */

#ifndef SRC_ENCODERMOTORCONTROL_H_
#define SRC_ENCODERMOTORCONTROL_H_

#include <stdint.h>

#include <TB6612DcMotor.h>

/*
 * Encoders generate 110 Hz at max speed => 8 ms per period
 * since duty cycle is 1/3 choose 3 millis as ringing mask.
 * 3 ms means 2.1 to 3.0 ms depending on the ms trigger.
 */
#define ENCODER_SENSOR_MASK_MILLIS 3
#define VELOCITY_SCALE_VALUE 500

/*
 * Default values if EEPROM values are invalid
 */
#define DEFAULT_MIN_SPEED   45
#define DEFAULT_STOP_SPEED  50
#define DEFAULT_MAX_SPEED   80

/*
 * Motor Control
 */

// the smaller the value the steeper the ramp
#define RAMP_UP_UPDATE_INTERVAL_MILLIS 16
// gives 16 steps a 16 millis for ramp up => 256 milliseconds
#define RAMP_UP_VALUE_DELTA ((ActualMaxSpeed - MinSpeed) / 16)

// timeout after ramp down was finished to switch off motor
#define RAMP_DOWN_TIMEOUT_MILLIS 500

// Ticks for ramp down if external stop requested
#define RAMP_DOWN_MIN_TICKS 3

// Safety net. If difference between targetCount and current distanceCount is less than, adjust new targetCount
#define MAX_DISTANCE_DELTA 8

#define MOTOR_STATE_STOPPED 0
#define MOTOR_STATE_RAMP_UP 1
#define MOTOR_STATE_FULL_SPEED 2
#define MOTOR_STATE_RAMP_DOWN 3

//#define MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS 500
#define MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS 100

#define SERVO_CURRENT_LOW_THRESHOLD 100
#define SERVO_INITIAL_DELAY 5

#define SERVO_CURRENT_LOW_MILLIS_FOR_SERVO_STOPPED 12

struct EepromMotorInfoStruct {
    uint8_t MinSpeed;
    uint8_t StopSpeed;
    uint8_t MaxSpeed;
    uint8_t SpeedCompensation;
};

class EncoderMotor: TB6612DcMotor {
public:

    EncoderMotor();
//    virtual ~EncoderMotor();

    void init(uint8_t aMotorNumber);

    /*
     * Set values for going a fixed distance
     */
    void initGoDistanceCount(int aDistanceCount);
    void updateMotor();

    /*
     * Setting DC Motor values direct, speed == 0 also resets other controls
     */
    void setSpeedCompensated(uint8_t aRequestedSpeed);
    void setDirection(bool goForward);

    /*
     * Sets speed to 0 and activate motor control for the appropriate direction
     */
    void activate();
    /*
     * Sets speed to 0 and deactivate motor and maintain control variables
     */
    void shutdownMotor(bool doBrake);
    /*
     * Shutdown and reset all control values and sets direction to forward
     */
    void resetAndShutdown();

    /*
     * Computes motor speed compensation value in order to go exactly straight ahead
     */
    void synchronizeMotor(EncoderMotor * aOtherMotorControl, uint16_t aCheckInterval);
    /*
     * Generates a rising ramp and detects the first movement -> this sets dead band / minimum Speed
     */
    static void calibrate();
    /*
     * Read and store calibration and other control values (maxSpeed, SpeedCompensation) from/to EEPROM
     */
    void readEeprom();
    void writeEeprom();

    /*
     * Encoder interrupt handling
     */
    void handleEncoderInterrupt();
    static void enableBothInterruptsOnBothEdges();
    static void enableInterruptOnBothEdges(uint8_t aIntPinNumber);

    /*
     * Static convenience functions. If you have 2 motors, better use CarControl
     */
    static void updateAllMotors();
    static void setDirectionForAll(bool goForward);

    static void initGoDistanceCountForAll(int aDistanceCount);
    static void goDistanceCountForAll(int aDistanceCount, void (*aLoopCallback)(void));
    static void startAndWaitForFullSpeedForAll(void);

    static void shutdownAllMotors(bool doBrake);
    static void resetAndStopAllMotors();
    static void waitUntilAllMotorsStopped(void (*aLoopCallback)(void));
    static void stopAllMotorsAndWaitUntilStopped();

    static bool allMotorsStarted();
    static bool allMotorsStopped();

    /*
     * List for access to all motorControls
     */
    static EncoderMotor *sMotorControlListStart; // Root pointer to list of all motorControls
    static uint8_t sNumberOfMotorControls;
    // for display control
    volatile static bool DistanceTickCounterHasChanged;
    // for display control
    static bool ValuesHaveChanged;
    static bool EnableValuesPrint;

    EncoderMotor * NextMotorControl;

    // number of this object starting with 0. Used for eeprom storage of parameters of more than one motorControl.
    uint8_t EncoderMotorNumber;

    /*
     * Minimum speed setting at which motor starts moving. Depend on current voltage, load and surface.
     * Is computed by calibrate() and then stored (with the other values) in eeprom.
     */
    uint8_t MinSpeed;
    /*
     * minimum speed to approach target count in order to be able to stop fast - currently set to MinSpeed
     */
    uint8_t StopSpeed;
    // maximum speed - TODO 0xFF should be working
    uint8_t MaxSpeed;
    // positive value to be subtracted from TargetSpeed to get ActualSpeed to compensate for different left and right motors
    // currently SpeedCompensation is in steps of 2 and only one motor can have a positive value, the other has zero.
    uint8_t SpeedCompensation;

    /*
     * Reset() resets all members from ActualSpeed to (including) Debug to 0
     */
    uint8_t ActualSpeed;
    volatile int16_t ActualVelocity;
    // can be set for eg. turning which better performs with reduced max speed
    uint8_t ActualMaxSpeed;
    //
    bool isDirectionForward;

    uint8_t State;
    uint16_t TargetDistanceCount;
    uint16_t LastTargetDistanceCount;
    // count of last ride - from start of MOTOR_STATE_RAMP_UP to next MOTOR_STATE_RAMP_UP
    uint16_t LastRideDistanceCount;

    /*
     * Distance optocoupler impulse counter. It is reset at initGoDistanceCount if motor was stopped.
     */
    volatile uint16_t DistanceCount;
    // used for debouncing and lock/timeout  detection
    unsigned long DistanceTickLastMillis;

    /*
     * For ramp and state control
     */
    uint8_t RampDelta;
    uint8_t RampDeltaPerDistanceCount;
    // target count at which next change must be done
    uint16_t NextChangeMaxTargetCount;
    unsigned long NextRampChangeMillis;

    // number of ticks at the transition from MOTOR_STATE_RAMP_UP to MOTOR_STATE_FULL_SPEED to be used for computing ramp down start ticks
    uint8_t DistanceCountAfterRampUp;
    uint16_t DebugCount;
    uint8_t SpeedAtTargetCountReached;

    // do not delete it!!! It must be the last element in structure and is needed for resetAndStop()
    uint16_t Debug;

};

#endif /* SRC_ENCODERMOTORCONTROL_H_ */

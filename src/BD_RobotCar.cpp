/*
 *  BD_RobotCar.cpp
 *
 *  Enables Robot car control with the BlueDisplay app.
 *
 *  Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
 *  To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the area.
 *  Manual control is by a GUI implemented with a Bluetooth HC-05 Module and the BlueDisplay library.
 *  Just overwrite the 2 functions myOwnFillForwardDistancesInfo() and doUserCollisionDetection() to test your own skill.
 *
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "RobotCar.h"
#include "RobotCarGui.h"
#include "Distance.h"

#include "HCSR04.h"
#include "ADCUtils.h"

#ifdef USE_TB6612_BREAKOUT_BOARD
#include <PlayRtttl.h>
#endif

#define VERSION_EXAMPLE "2.0"

/*
 * Car Control
 */
CarMotorControl RobotCar;
float sVINVoltage;
void checkForLowVoltage();

#ifdef ENABLE_RTTTL
bool sPlayMelody = false;
void playRandomMelody();
#endif

void initLaserServos();
void doAutonomousDrive();

/*************************************************************************************
 * Extend this basic collision detection to test your own skill in autonomous driving
 *
 * Checks distances and returns degree to turn
 * @return 0 -> no turn, >0 -> turn left, <0 -> turn right
 *************************************************************************************/
int doUserCollisionDetection() {
// if left three distances are all less than 21 centimeter then turn right.
    if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT - 1] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT - 2] <= MINIMUM_DISTANCE_TO_SIDE) {
        return -90;
        // check right three distances are all less then 21 centimeter than turn left.
    } else if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT + 1] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT + 2] <= MINIMUM_DISTANCE_TO_SIDE) {
        return 90;
        // check front distance is longer then 35 centimeter than do not turn.
    } else if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_1] >= MINIMUM_DISTANCE_TO_FRONT
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_2] >= MINIMUM_DISTANCE_TO_FRONT) {
        return 0;
    } else if (sForwardDistancesInfo.MaxDistance >= MINIMUM_DISTANCE_TO_SIDE) {
        /*
         * here front distance is less then 35 centimeter:
         * go to max side distance
         */
        // formula to convert index to degree.
        return -90 + DEGREES_PER_STEP * sForwardDistancesInfo.IndexOfMaxDistance;
    } else {
        // Turn backwards.
        return 180;
    }
}

/*
 * Start of robot car control program
 */
void setup() {
// initialize digital pins as an output.
    pinMode(TRIGGER_OUT_PIN, OUTPUT);

#ifdef USE_US_SENSOR_1_PIN_MODE
    initUSDistancePin(TRIGGER_OUT_PIN);
#else
    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);
#endif
    initUSServo();

#ifdef HAS_LASER
    pinMode(LASER_OUT_PIN, OUTPUT);
    initLaserServos();
#endif

#ifdef USE_TB6612_BREAKOUT_BOARD
    pinMode(CAMERA_SUPPLY_CONTROL_PIN, OUTPUT);
#endif

    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    EncoderMotor::enableBothInterruptsOnBothEdges();
    EncoderMotor::EnableValuesPrint = true;
    // initialize motors
    RobotCar.init(TWO_WD_DETECTION_PIN);

// reset all values
    resetPathData();

    setupGUI();

    // Just to know which program is running on my Arduino
    BlueDisplay1.debug("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);

    initDistance();

    readVINVoltage();
    randomSeed(sVINVoltage * 10000);
    tone(SPEAKER_PIN, 2200, 100);
}

void loop() {
    checkForLowVoltage();

    /*
     * check if timeout, no Bluetooth connection and connected to LIPO battery
     */
    if ((!BlueDisplay1.isConnectionEstablished()) && (millis() < 11000) && (millis() > 10000)
            && (sVINVoltage > VOLTAGE_USB_THRESHOLD)) {
        /*
         * Timeout just reached, play melody and start autonomous drive
         */
#ifdef ENABLE_RTTTL
        playRandomMelody();
        delayAndLoopGUI(1000);
#else
        delayAndLoopGUI(6000); // delay needed for millis() check above!
#endif
        startStopAutomomousDrive(true, true);
    }

    /*
     * check for user input and update display output
     */
    loopGUI();

    /*
     * After 4 minutes of user inactivity, make noise by scanning with US Servo and repeat it every 2. minute
     */
#if VERSION_BLUE_DISPLAY_NUMERICAL >= 121
    if (BlueDisplay1.isConnectionEstablished() && sMillisOfLastReceivedBDEvent + 240000L < millis()) {
        sMillisOfLastReceivedBDEvent = millis() - 120000; // adjust sMillisOfLastReceivedBDEvent to have the next scan in 2 minutes
        fillAndShowForwardDistancesInfo((sCurrentPage == PAGE_AUTOMATIC_CONTROL), true);
        DistanceServo.write(90); // set servo back to normal
    }
#endif

#ifdef ENABLE_RTTTL
    /*
     * check for playing melody
     */
    if (sPlayMelody) {
        RobotCar.resetAndShutdownMotors();
        playRandomMelody();
    }

#endif

    if (sRobotCarStarted && (sCurrentPage == PAGE_HOME || sCurrentPage == PAGE_TEST)) {
        /*
         * Direct speed control by GUI
         */
        RobotCar.updateMotors();
        rightEncoderMotor.synchronizeMotor(&leftEncoderMotor, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
    }

    if (sRuningAutonomousDrive) {
        /*
         * Start autonomous driving
         */
        doAutonomousDrive();
    }
}

/*
 * Start, loop and stop of autonomous driving
 */
void doAutonomousDrive() {
    /*
     * Start autonomous driving
     */
    int (*tCollisionDetectionFunction)();

    if (sUseBuiltInAutonomousDriveStrategy) {
        tCollisionDetectionFunction = &doBuiltInCollisionDetection;
    } else {
        tCollisionDetectionFunction = &doUserCollisionDetection;
    }
    EncoderMotor::EnableValuesPrint = false;
    /*
     * Autonomous driving main loop
     */
    while (sRuningAutonomousDrive) {
        driveAutonomousOneStep(tCollisionDetectionFunction);
        /*
         * check for user input and update display output
         */
        loopGUI();
    }
    /*
     * Stop autonomous driving. RobotCar.isStopped() is true here
     */
    if (sStepMode != MODE_SINGLE_STEP) {
        // add last driven distance to path
        insertToPath(rightEncoderMotor.LastRideDistanceCount, sLastDegreesTurned, true);
    }
    EncoderMotor::EnableValuesPrint = true;
    DistanceServoWriteAndDelay(90);
}

void readVINVoltage() {
    uint8_t tOldADMUX = checkAndWaitForReferenceAndChannelToSwitch(VIN_11TH_IN_CHANNEL, INTERNAL);
    float tVIN = readADCChannelWithReferenceOversample(VIN_11TH_IN_CHANNEL, INTERNAL, 2); // 4 samples
    // Switch back (to DEFAULT)
    ADMUX = tOldADMUX;

// assume resistor network of 100k / 10k (divider by 11)
// tVCC * 0,01181640625
#ifdef USE_TB6612_BREAKOUT_BOARD
    // we have a Diode (needs 0.8 volt) between LIPO and VIN
    sVINVoltage = (tVIN * ((11.0 * 1.07) / 1023)) + 0.8;
#else
    sVINVoltage = tVIN * ((11.0 * 1.07) / 1023);
#endif
}

void checkForLowVoltage() {
    if (sVINVoltage < VOLTAGE_LOW_THRESHOLD && sVINVoltage > VOLTAGE_USB_THRESHOLD) {
        drawCommonGui();
        uint16_t tNextX = BlueDisplay1.drawText(10, 50, F("Battery voltage"), TEXT_SIZE_33, COLOR_RED, COLOR_WHITE);
        char tDataBuffer[18];
        char tVCCString[6];
        dtostrf(sVINVoltage, 4, 2, tVCCString);
        sprintf_P(tDataBuffer, PSTR("%s volt"), tVCCString);
        BlueDisplay1.drawText(tNextX + TEXT_SIZE_33_WIDTH, 50, tDataBuffer);
        // alternatively call readAndPrintVinPeriodically();

        BlueDisplay1.drawText(10 + (4 * TEXT_SIZE_33_WIDTH), 50 + TEXT_SIZE_33_HEIGHT, F("too low"));
        RobotCar.resetAndShutdownMotors();
        do {
            readAndPrintVinPeriodically();
            delay(PRINT_VOLTAGE_PERIOD_MILLIS);
        } while (sVINVoltage < VOLTAGE_LOW_THRESHOLD && sVINVoltage > VOLTAGE_USB_THRESHOLD);
        // refresh current page
        GUISwitchPages(NULL, 0);
    }
}

#ifdef ENABLE_RTTTL
#include "digitalWriteFast.h"
/*
 * Prepare for tone, use motor as loudspeaker
 */
void playRandomMelody() {
    // this flag may be reseted by checkAndHandleEvents()
    sPlayMelody = true;
    BlueDisplay1.debug("Play melody");

    OCR2B = 0;
    bitWrite(TIMSK2, OCIE2B, 1); // enable interrupt for inverted pin handling
    startPlayRandomRtttlFromArrayPGM(MOTOR_0_FORWARD_PIN, RTTTLMelodiesSmall, ARRAY_SIZE_MELODIES_SMALL);
    while (updatePlayRtttl()) {
        // check for pause in melody (i.e. timer disabled) and disable motor for this period
        if ( TIMSK2 & _BV(OCIE2A)) {
            // timer enabled
            digitalWriteFast(MOTOR_0_PWM_PIN, HIGH); // re-enable motor
        } else {
            // timer disabled
            digitalWriteFast(MOTOR_0_PWM_PIN, LOW); // disable motor for pause in melody
        }
        checkAndHandleEvents();
        if (!sPlayMelody) {
            BlueDisplay1.debug("Stop melody");
            break;
        }
    }
    TouchButtonMelody.setValue(false, (sCurrentPage == PAGE_HOME));
    digitalWriteFast(MOTOR_0_PWM_PIN, LOW); // disable motor
    bitWrite(TIMSK2, OCIE2B, 0); // disable interrupt
    sPlayMelody = false;
}

void playTone(unsigned int aFrequency, unsigned long aDuration = 0) {
    OCR2B = 0;
    bitWrite(TIMSK2, OCIE2B, 1); // enable interrupt for inverted pin handling
    tone(MOTOR_0_FORWARD_PIN, aFrequency);
    delay(aDuration);
    noTone(MOTOR_0_FORWARD_PIN);
    digitalWriteFast(MOTOR_0_PWM_PIN, LOW); // disable motor
    bitWrite(TIMSK2, OCIE2B, 0); // disable interrupt
}

/*
 * set INVERTED_TONE_PIN to inverse value of TONE_PIN to avoid DC current
 */
#ifdef USE_TB6612_BREAKOUT_BOARD
ISR(TIMER2_COMPB_vect) {
    digitalWriteFast(MOTOR_0_BACKWARD_PIN, !digitalReadFast(MOTOR_0_FORWARD_PIN));
}
#endif
#endif // ENABLE_RTTTL

/*
 * Laser servo stuff
 */
Servo LaserPanServo;
#ifdef USE_PAN_TILT_SERVO
Servo TiltServo;
#endif

void initLaserServos() {
#ifdef USE_PAN_TILT_SERVO
    TiltServo.attach(LASER_SERVO_TILT_PIN);
    TiltServo.write(TILT_SERVO_MIN_VALUE); // my servo makes noise at 0 degree.
#endif
// initialize and set Laser pan servo
    LaserPanServo.attach(LASER_SERVO_PAN_PIN);
    LaserPanServo.write(90);
}


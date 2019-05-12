/*
 * RobotCarGui.cpp
 *
 *  Contains all the GUI elements for operating and controlling the RobotCar.
 *  Calibration: Sets lowest speed for which wheels are moving.
 *  Speed Slider left: Sets speed for manual control which serves also as maximum speed for autonomous drive if "Stored"
 *  Store: Stores calibration info and maximum speed.
 *  Cont ->Step / Step -> SStep, SStep->Cont: Switches mode from "continuous drive" to "drive until next turn" to "drive CENTIMETER_PER_RIDE_PRO"
 *  Start Simple: Start simple driving algorithm (using the 2 "simple" functions in RobotCar.cpp)
 *  Start Pro: Start elaborated driving algorithm
 *
 *  insertToPath() and DrawPath() to show the path we were driving.
 *
 *  Needs BlueDisplay library.
 *
 *  Created on: 20.09.2016
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include "RobotCarGui.h"
#include "RobotCar.h"
#include "AutonomousDrive.h"
#include <HCSR04.h>

#include <avr/interrupt.h>

/*
 * Motor GUI
 */
BDButton TouchButtonRobotCarStartStop;
BDButton TouchButtonReset;
BDButton TouchButtonBack;
BDButton TouchButtonResetPath;
BDButton TouchButtonStartStopAutonomousForPathPage;

BDButton TouchButtonNextPage;
BDButton TouchButtonDirection;
BDButton TouchButtonSetSpeed;

BDButton TouchButtonTestOwn;
BDButton TouchButtonAutonomousDrive;
BDButton TouchButtonStepMode;
BDButton TouchButtonSingleStep;
BDButton TouchButtonSingleScan;

BDButton TouchButtonCalibrate;
BDButton TouchButtonDebug;

BDButton TouchButton5cm;
BDButton TouchButton10cm;
BDButton TouchButton20cm;
BDButton TouchButton40cm;

BDButton TouchButton45DegreeRight;
BDButton TouchButton45DegreeLeft;
BDButton TouchButton90DegreeRight;
BDButton TouchButton90DegreeLeft;
BDButton TouchButton360Degree;

BDSlider SliderSpeed;
uint16_t sLastSpeedSliderValue = 0;

BDSlider SliderSpeedRight;
BDSlider SliderSpeedLeft;

uint8_t sActualPage;

bool sDoStep; // if true => do one step

bool sStarted;
bool sRunOwnTest;
bool sRunAutonomousDrive;
bool sLastRunWasAutonomousDrive = true;

bool sShowDebug = false;

/*
 * UltraSonic control GUI
 */
BDSlider SliderUSPosition;
BDSlider SliderUSDistance;
unsigned int sSliderLastCentimeter;

unsigned int sLastCentimeterToObstacle = 0;
const int sGetDistancePeriod = 500;

uint8_t sStepMode = MODE_CONTINUOUS;
void setStepModeButtonCaption();
void setTestOwnButtonValue();
void setAutonomousDriveButtonValue();
void setStartStopAutonomousForPathPageButtonValue();

// a string buffer for any purpose...
char sStringBuffer[128];

void setupGUI(void) {
#ifdef USE_SIMPLE_SERIAL  // see line 38 in BlueSerial.h - use global #define USE_STANDARD_SERIAL to disable it
    initSimpleSerial(HC_05_BAUD_RATE);
#else
    Serial.begin(HC_05_BAUD_RATE);
#endif

    sActualPage = PAGE_MANUAL_CONTROL;

    // Register callback handler and check for connection
    BlueDisplay1.initCommunication(&initDisplay, &drawGui);
}

void loopGUI(void) {

    // do not show anything during motor speed ramps
    if (myCar.isState(MOTOR_STATE_STOPPED) || myCar.isState(MOTOR_STATE_FULL_SPEED)) {

        /*
         * Display changed values in GUI only at manual page
         */
        if (sActualPage == PAGE_MANUAL_CONTROL) {
            checkAndShowDistancePeriodically(sGetDistancePeriod);

            uint8_t ActualRequestedSpeed = myCar.rightMotorControl.ActualSpeed + myCar.rightMotorControl.SpeedCompensation;
            if (ActualRequestedSpeed != sLastSpeedSliderValue) {
                SliderSpeed.setActualValueAndDrawBar(ActualRequestedSpeed);
                sLastSpeedSliderValue = ActualRequestedSpeed;
            }
            if (EncoderMotorControl::ValuesHaveChanged) {
                EncoderMotorControl::ValuesHaveChanged = false;
                printMotorValues();
                if (sShowDebug) {
                    printMotorDebugValues();
                }
            }

            /*
             * Display velocity as slider values
             */
            EncoderMotorControl * tMotorInfo = &myCar.leftMotorControl;
            BDSlider * tSliderPtr = &SliderSpeedLeft;
            uint16_t tXPos = 0;
            for (int i = 0; i < 2; ++i) {
                if (EncoderMotorControl::DistanceTickCounterHasChanged) {
                    tSliderPtr->setActualValueAndDrawBar(tMotorInfo->ActualVelocity);
                }
                tMotorInfo = &myCar.rightMotorControl;
                tSliderPtr = &SliderSpeedRight;
                tXPos += BUTTON_WIDTH_16 + 4;
            }
        }

        if (sActualPage != PAGE_SHOW_PATH) {
            /*
             * Print changed tick values
             */
            if (EncoderMotorControl::DistanceTickCounterHasChanged) {
                EncoderMotorControl::DistanceTickCounterHasChanged = false;
                printDistanceValues();
            }

            printVinPeriodically();
        }
    }

    /*
     * Check if receive buffer contains an event
     * Do it at end since it may change the condition for calling this function and its printing
     */
    checkAndHandleEvents();
}

void doDistance(BDButton * aTheTouchedButton, int16_t aValue) {
    if (!myCar.isDirectionForward) {
        aValue = -aValue;
    }
    myCar.initGoDistanceCentimeter(aValue);
}

void doShowDebug(BDButton * aTheTouchedButton, int16_t aValue) {
    sShowDebug = aValue;
}

void doCalibrate(BDButton * aTheTouchedButton, int16_t aValue) {
    EncoderMotorControl::calibrate();
}

/*
 * Own test starts in mode SINGLE_STEP
 */
void doStartStopTestOwn(BDButton * aTheTouchedButton, int16_t aValue) {
    sRunOwnTest = aValue;
    // copy state to pathPage Button
    TouchButtonStartStopAutonomousForPathPage.setValue(aValue);

    sLastRunWasAutonomousDrive = false;

    if (sRunOwnTest) {
        sRunAutonomousDrive = false;
        // disable other button
        setAutonomousDriveButtonValue();

        resetPathData();
        sStepMode = MODE_SINGLE_STEP;
        setStepModeButtonCaption();
        TouchButtonStepMode.drawButton();
        // enable motors
        myCar.activateMotors();
    } else {
        // stop, but preserve direction
        myCar.shutdownMotors(false);
    }
}

void doStartStopAutomomousDrive(BDButton * aTheTouchedButton, int16_t aValue) {
    sRunAutonomousDrive = aValue;
    // copy state to pathPage Button
    TouchButtonStartStopAutonomousForPathPage.setValue(aValue);

    sLastRunWasAutonomousDrive = true;

    if (sRunAutonomousDrive) {
        // disable other button
        sRunOwnTest = false;
        setTestOwnButtonValue();

        resetPathData();
        // enable motors
        myCar.activateMotors();
    } else {
        // stop, but preserve direction
        myCar.shutdownMotors(false);
    }
}

/*
 * Handle Start/Stop for Path page - start stop last test that was run.
 */
void doStartStopAutonomousForPathPage(BDButton * aTheTouchedButton, int16_t aValue) {
    if (sLastRunWasAutonomousDrive) {
        sRunAutonomousDrive = aValue;
        setAutonomousDriveButtonValue();
    } else {
        sRunOwnTest = aValue;
        setTestOwnButtonValue();
    }
    if (aValue) {
        sDoStep = true;
    } else {
        // stop, but preserve direction
        myCar.shutdownMotors(false);
    }
}

void doRotation(BDButton * aTheTouchedButton, int16_t aValue) {
    myCar.initRotateCar(aValue, !myCar.isDirectionForward);
}

/*
 * Store ActualSpeed as MaxSpeed
 */
void doSetSpeed(float aValue) {
    uint16_t tValue = aValue;
    if (tValue > 10 && tValue < 256) {
        // must use value for compensation not compensated value
        myCar.rightMotorControl.MaxSpeed = tValue;
        myCar.rightMotorControl.writeEeprom();

        // use the same value here !
        myCar.leftMotorControl.MaxSpeed = tValue;
        myCar.leftMotorControl.writeEeprom();
    }
    printMotorValues();
}

/*
 * Request speed value as number
 */
void doGetSpeed(BDButton * aTheTouchedButton, int16_t aValue) {
    BlueDisplay1.getNumberWithShortPrompt(&doSetSpeed, "Speed [11 - 255]");
}

void speedSliderSetValue(uint8_t aSpeed, bool aUpdateBar) {
    if (aUpdateBar) {
        SliderSpeed.setActualValueAndDrawBar(aSpeed);
    }
    sprintf_P(sStringBuffer, PSTR("%3d"), aSpeed);
    SliderSpeed.printValue(sStringBuffer);
}

/*
 * Minimum Speed is 30 for USB Power and no load 50 for load
 * Minimum Speed is 20 for 2 Lithium 18650 battery Power and no load 25 for load
 */
void doSpeedSlider(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    if (aValue != sLastSpeedSliderValue) {
        sLastSpeedSliderValue = aValue;
        myCar.setSpeedCompensated(aValue);
    }
}

/*
 * Convert full range to 180 degree
 */
void doUSPosition(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    US_ServoWrite(aValue);
}

void setStartStopButtonValue() {
    TouchButtonRobotCarStartStop.setValueAndDraw(sStarted);
}

void setStartStopAutonomousForPathPageButtonValue() {
    TouchButtonStartStopAutonomousForPathPage.setValueAndDraw(sStarted);
}

void setTestOwnButtonValue() {
    TouchButtonTestOwn.setValue(sRunOwnTest, (sActualPage == PAGE_AUTOMATIC_CONTROL));
}

void setAutonomousDriveButtonValue() {
    TouchButtonAutonomousDrive.setValue(sRunAutonomousDrive, (sActualPage == PAGE_AUTOMATIC_CONTROL));
}

void setSwitchPagesButtonCaption() {
    if (sActualPage == PAGE_MANUAL_CONTROL) {
        TouchButtonNextPage.setCaptionPGM(PSTR("Automatic\nControl"));
    } else {
        TouchButtonNextPage.setCaptionPGM(PSTR("Show\nPath"));
    }
}

/*
 * Handle Start/Stop
 */
void doRobotCarStartStop(BDButton * aTheTouchedButton, int16_t aValue) {
    sStarted = !sStarted;
    if (sStarted) {
        // enable motors
        myCar.activateMotors();
        setStartStopButtonValue();
    } else {
        // stop, but preserve direction
        myCar.shutdownMotors(false);
        // reset also flags etc
        resetGUIControls();
    }
    SliderSpeed.setActualValueAndDrawBar(0);
}

void setStepModeButtonCaption() {
    if (sStepMode == MODE_CONTINUOUS) {
        TouchButtonStepMode.setCaptionPGM(PSTR("Cont\n->Step"));
    } else if (sStepMode == MODE_STEP) {
        TouchButtonStepMode.setCaptionPGM(PSTR("Step\n->SStep"));
    } else {
        TouchButtonStepMode.setCaptionPGM(PSTR("SStep\n->Cont"));
    }
}

/*
 * Switches modes MODE_CONTINUOUS -> MODE_STEP -> MODE_SINGLE_STEP
 */
void doStepMode(BDButton * aTheTouchedButton, int16_t aValue) {
    sStepMode++;
    if (sStepMode == MODE_SINGLE_STEP) {
        myCar.stopCar();
    } else if (sStepMode > MODE_SINGLE_STEP) {
        sStepMode = MODE_CONTINUOUS;
    }
    setStepModeButtonCaption();
    TouchButtonStepMode.drawButton();
}

/*
 * enables next step
 */
void doSingleStep(BDButton * aTheTouchedButton, int16_t aValue) {
    if (sStepMode == MODE_CONTINUOUS) {
        // switch to step mode MODE_SINGLE_STEP
        sStepMode++;
        doStepMode(NULL, 0);
    }
    sDoStep = true;
}

void doSingleScan(BDButton * aTheTouchedButton, int16_t aValue) {
    bool tInfoWasProcessed;
    if (sRunOwnTest) {
        tInfoWasProcessed = myOwnFillForwardDistancesInfo(&ForwardDistancesInfo, true, true);
    } else {
        clearPrintedForwardDistancesInfos();
        tInfoWasProcessed = fillForwardDistancesInfo(&ForwardDistancesInfo, true, true);
        doWallDetection(&ForwardDistancesInfo, true);
        sNextDegreesToTurn = doCollisionDetectionPro(&ForwardDistancesInfo);
        drawCollisionDecision(sNextDegreesToTurn, CENTIMETER_PER_RIDE_PRO, false);
    }
    if (!tInfoWasProcessed) {
        drawForwardDistancesInfos(&ForwardDistancesInfo);
    }
}

void setDirectionButtonCaption() {
    if (myCar.isDirectionForward) {
// direction forward
        TouchButtonDirection.setCaption("\x87");
    } else {
// direction backward
        TouchButtonDirection.setCaption("\x88");
    }
}

/*
 * Stops motors and change direction
 */
void GUIchangeDirection(BDButton * aTheTouchedButton, int16_t aValue) {
    myCar.setDirection(!myCar.isDirectionForward);
    setDirectionButtonCaption();
    TouchButtonDirection.drawButton();

    sStarted = false;
    setStartStopButtonValue();
    SliderSpeed.setActualValueAndDrawBar(0);
}

void GUISwitchPages(BDButton * aTheTouchedButton, int16_t aValue) {
    sActualPage++;
    if (sActualPage > PAGE_SHOW_PATH) {
        sActualPage = PAGE_MANUAL_CONTROL;
    }
    drawGui();
}

/*
 * Handle Back Button
 */
void doBack(BDButton * aTheTouchedButton, int16_t aValue) {
    sActualPage--;
    drawGui();
}

void doReset(BDButton * aTheTouchedButton, int16_t aValue) {
    resetGUIControls();
    resetPathData();
}

void doResetPath(BDButton * aTheTouchedButton, int16_t aValue) {
    resetPathData();
    DrawPath();
}

void initDisplay(void) {
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_TOUCH_BASIC_DISABLE | BD_FLAG_USE_MAX_SIZE, DISPLAY_WIDTH,
    DISPLAY_HEIGHT);
    BlueDisplay1.setCharacterMapping(0x87, 0x2227); // mapping for AND - Forward
    BlueDisplay1.setCharacterMapping(0x88, 0x2228); // mapping for OR - Backwards

    /*
     * Speed Sliders
     */
    SliderSpeed.init(0, 10, BUTTON_WIDTH_6, SPEED_SLIDER_SIZE, 200, 0, COLOR_YELLOW,
    SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doSpeedSlider);
    SliderSpeed.setScaleFactor(255.0 / SPEED_SLIDER_SIZE); // Slider is virtually 2 times larger

    SliderSpeedLeft.init(BUTTON_WIDTH_6 + 4, 0, BUTTON_WIDTH_16, SPEED_SLIDER_SIZE / 2, SPEED_SLIDER_SIZE / 2 - 1, 0,
    SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);

    SliderSpeedRight.init(BUTTON_WIDTH_6 + 4 + BUTTON_WIDTH_16 + 8, 0, BUTTON_WIDTH_16, SPEED_SLIDER_SIZE / 2,
    SPEED_SLIDER_SIZE / 2 - 1, 0, SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);

    /*
     * scaled US Sliders
     */

    SliderUSDistance.init(BUTTON_WIDTH_6_POS_6 - BUTTON_WIDTH_10 - 4, 10, BUTTON_WIDTH_10, US_SLIDER_SIZE, 200, 0,
    SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT,
    NULL);
    SliderUSDistance.setScaleFactor(2);
    SliderUSDistance.setValueUnitString("cm");

    SliderUSPosition.init(BUTTON_WIDTH_6_POS_6, 10, BUTTON_WIDTH_6, US_SLIDER_SIZE, 90, 90, COLOR_YELLOW, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE, &doUSPosition);
    SliderUSPosition.setBarThresholdColor(COLOR_BLUE);
    SliderUSPosition.setScaleFactor(180.0 / US_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderUSPosition.setValueUnitString("\xB0");

    /*
     * Control buttons
     */
    TouchButtonBack.initPGM(BUTTON_WIDTH_4_POS_4, 0, BUTTON_WIDTH_4, BUTTON_HEIGHT_4, COLOR_RED, PSTR("Back"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doBack);

    TouchButtonResetPath.initPGM(0, 0, BUTTON_WIDTH_4, BUTTON_HEIGHT_4, COLOR_RED, PSTR("Clear"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doResetPath);

    TouchButtonRobotCarStartStop.initPGM(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_BLUE, PSTR("Start"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sStarted, &doRobotCarStartStop);
    TouchButtonRobotCarStartStop.setCaptionPGMForValueTrue(PSTR("Stop"));

    TouchButtonReset.initPGM(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_BLUE, PSTR("Reset"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doReset);

    TouchButtonNextPage.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, "",
    TEXT_SIZE_16, FLAG_BUTTON_DO_BEEP_ON_TOUCH, myCar.isDirectionForward, &GUISwitchPages);
    setSwitchPagesButtonCaption();

    TouchButtonSetSpeed.initPGM(0, BUTTON_HEIGHT_4_LINE_4 - BUTTON_HEIGHT_8_LINE_2 + 1, BUTTON_WIDTH_6, BUTTON_HEIGHT_8,
    COLOR_BLUE, PSTR("Set\nspeed"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doGetSpeed);

    TouchButtonStepMode.init(0, 0, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_4, COLOR_BLUE, "", TEXT_SIZE_18, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0,
            &doStepMode);
    setStepModeButtonCaption();

    TouchButtonSingleScan.initPGM(0, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_4, COLOR_BLUE, PSTR("Scan"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doSingleScan);

    TouchButtonSingleStep.initPGM(0, BUTTON_HEIGHT_4_LINE_3, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_4, COLOR_BLUE, PSTR("Step"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doSingleStep);

    TouchButtonAutonomousDrive.initPGM(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, PSTR("Start"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sRunAutonomousDrive,
            &doStartStopAutomomousDrive);
    TouchButtonAutonomousDrive.setCaptionPGMForValueTrue(PSTR("Stop"));

    TouchButtonTestOwn.initPGM(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED,
            PSTR("Start\nyour own"),
            TEXT_SIZE_18, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sRunOwnTest, &doStartStopTestOwn);
    TouchButtonTestOwn.setCaptionPGMForValueTrue(PSTR("Stop\nyour own"));

    // copy of button TouchButtonAutonomousDrive for Path page
    TouchButtonStartStopAutonomousForPathPage.initPGM(BUTTON_WIDTH_4_POS_4, 0, BUTTON_WIDTH_4, BUTTON_HEIGHT_4, COLOR_RED,
            PSTR("Start"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sStarted,
            &doStartStopAutonomousForPathPage);
    TouchButtonStartStopAutonomousForPathPage.setCaptionPGMForValueTrue(PSTR("Stop"));

    /*
     * Test buttons
     */
    TouchButton5cm.initPGM(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, PSTR("5cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 5, &doDistance);
    TouchButton10cm.initPGM(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, PSTR("10cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 10, &doDistance);

    TouchButton20cm.initPGM(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, PSTR("20cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 20, &doDistance);
    TouchButton40cm.initPGM(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, PSTR("40cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 40, &doDistance);

    TouchButton45DegreeLeft.initPGM(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("45\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 45, &doRotation);
    TouchButton45DegreeRight.initPGM(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("45\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, -45, &doRotation);

    TouchButton90DegreeLeft.initPGM(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("90\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 90, &doRotation);
    TouchButton90DegreeRight.initPGM(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("90\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, -90, &doRotation);
    TouchButton360Degree.initPGM(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("360\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 360, &doRotation);

    TouchButtonCalibrate.initPGM(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_RED,
            PSTR("CAL"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doCalibrate);

    TouchButtonDebug.initPGM(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_RED, PSTR("dbg"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sShowDebug, &doShowDebug);

    // Direction
    TouchButtonDirection.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, "",
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &GUIchangeDirection);
    setDirectionButtonCaption();
}

/*
 * Manual control page
 */
void drawGui(void) {
    BlueDisplay1.clearDisplay(COLOR_WHITE);

    BlueDisplay1.deactivateAllButtons();
    BlueDisplay1.deactivateAllSliders();

    setSwitchPagesButtonCaption();

    if (sActualPage != PAGE_SHOW_PATH) {
        BlueDisplay1.drawTextPGM(BUTTON_WIDTH_10_POS_4 - 8, TEXT_SIZE_22, PSTR("Robot Car Control"), TEXT_SIZE_16, COLOR_BLUE,
        COLOR_NO_BACKGROUND);
        TouchButtonNextPage.drawButton();
    }

    if (sActualPage == PAGE_MANUAL_CONTROL) {
        TouchButtonRobotCarStartStop.drawButton();
        TouchButtonReset.drawButton();

        TouchButtonDirection.drawButton();
        TouchButtonCalibrate.drawButton();
        TouchButtonDebug.drawButton();

        TouchButton5cm.drawButton();
        TouchButton10cm.drawButton();
        TouchButton20cm.drawButton();
        TouchButton40cm.drawButton();

        TouchButton45DegreeLeft.drawButton();
        TouchButton45DegreeRight.drawButton();
        TouchButton90DegreeLeft.drawButton();
        TouchButton90DegreeRight.drawButton();
        TouchButton360Degree.drawButton();

        SliderSpeed.drawSlider();
        SliderSpeedRight.drawSlider();
        SliderSpeedLeft.drawSlider();
        TouchButtonSetSpeed.drawButton();

        SliderUSPosition.setActualValueAndDrawBar(sLastServoAngleInDegrees);
        SliderUSPosition.drawSlider();
        SliderUSDistance.drawSlider();

        printMotorValues();
        if (sShowDebug) {
            printMotorDebugValues();
        }
        printDistanceValues();

    } else if (sActualPage == PAGE_AUTOMATIC_CONTROL) {

        TouchButtonBack.setPosition(BUTTON_WIDTH_4_POS_4, 0);
        TouchButtonBack.drawButton();

        TouchButtonTestOwn.drawButton();
        TouchButtonAutonomousDrive.drawButton();
        TouchButtonStepMode.drawButton();
        TouchButtonSingleScan.drawButton();
        TouchButtonSingleStep.drawButton();
        drawForwardDistancesInfos(&ForwardDistancesInfo);
        drawCollisionDecision(sNextDegreesToTurn, CENTIMETER_PER_RIDE_PRO, false);

    } else {
        // draws also the buttons, since it first clears the screen
        DrawPath();
    }
}

void resetGUIControls() {
    myCar.resetAndShutdownMotors();

    sRunOwnTest = false;
    sRunAutonomousDrive = false;

    TouchButtonTestOwn.setValue(sRunOwnTest, (sActualPage == PAGE_AUTOMATIC_CONTROL));
    TouchButtonAutonomousDrive.setValue(sRunAutonomousDrive, (sActualPage == PAGE_AUTOMATIC_CONTROL));

    sStarted = false;
    sDoStep = false;
    sStepMode = MODE_CONTINUOUS;

    setDirectionButtonCaption();
    setStartStopButtonValue();
    setStepModeButtonCaption();

    if (sActualPage == PAGE_MANUAL_CONTROL) {
        SliderSpeed.setActualValueAndDrawBar(0);
    }

}

void showDistance(unsigned int aCentimeter) {
// feedback as slider length
    if (aCentimeter != sSliderLastCentimeter) {
        sSliderLastCentimeter = aCentimeter;
        SliderUSDistance.setActualValueAndDrawBar(aCentimeter);
    }
}

void checkAndShowDistancePeriodically(uint16_t aPeriodMillis) {
    if (myCar.rightMotorControl.State != MOTOR_STATE_RAMP_DOWN && myCar.rightMotorControl.State != MOTOR_STATE_RAMP_UP
            && myCar.leftMotorControl.State != MOTOR_STATE_RAMP_DOWN && myCar.leftMotorControl.State != MOTOR_STATE_RAMP_UP) {
        static long sLastUSMeasurementMillis;
        long tMillis = millis();
        if (sLastUSMeasurementMillis + aPeriodMillis < tMillis) {
            sLastUSMeasurementMillis = tMillis;
            unsigned int tCentimeter = getUSDistanceAsCentiMeterWithCentimeterTimeout(300);
            // feedback as slider length
            if (tCentimeter != sLastCentimeterToObstacle) {
                sLastCentimeterToObstacle = tCentimeter;
                showDistance(tCentimeter);
            }
        }
    }
}

void printSingleDistanceVector(uint16_t aLength, int aDegrees, color16_t aColor) {
    BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, aLength, aDegrees, aColor, 3);
}

/*
 * Forward 0 degree is X direction
 */
int xPathDelta[PATH_LENGTH_MAX], yPathDelta[PATH_LENGTH_MAX];
int *sXPathDeltaPtr, *sYPathDeltaPtr;
// exact float values to avoid aggregating rounding errors
float sLastXPathFloat, sLastYPathFloat;
int sLastXPathInt, sLastYPathInt;
// for layout of path data
int sXPathMax, sXPathMin, sYPathMax, sYPathMin;
int sLastPathDirectionDegree;

void resetPathData() {
    sXPathDeltaPtr = &xPathDelta[0];
    sYPathDeltaPtr = &yPathDelta[0];
// set to origin
    sLastXPathFloat = 0.0;
    sLastYPathFloat = 0.0;
    sLastXPathInt = 0;
    sLastYPathInt = 0;
    sXPathMax = sYPathMax = 0;
    // Yes actual minimum is zero since it can be negative
    sXPathMin = sYPathMin = 0;

    sLastPathDirectionDegree = 0;
// clear delta arrays
    for (unsigned int i = 0; i < PATH_LENGTH_MAX; ++i) {
        xPathDelta[i] = 0;
        yPathDelta[i] = 0;
    }
    // to start new path in right direction
    sNextDegreesToTurn = 0;
    sLastDegreesTurned = 0;
}

/*
 * (Over-)writes to actual pointer position
 * 0 degree goes in X direction
 * @param aAddEntry if false only values of actual entry will be adjusted
 */
void insertToPath(int aLength, int aDegree, bool aAddEntry) {
//    BlueDisplay1.debug("Degree=", aDegree);
//    BlueDisplay1.debug("Length=", aLength);

    // get new direction
    int tLastPathDirectionDegree = sLastPathDirectionDegree + aDegree;
    if (aAddEntry) {
        sLastPathDirectionDegree = tLastPathDirectionDegree;
    }
//    BlueDisplay1.debug("LastDegree=", tLastPathDirectionDegree);

    float tRadianOfDegree = tLastPathDirectionDegree * (M_PI / 180);
    /*
     * compute X and Y delta and min/max
     */
    float tNewXPathFloat = (cos(tRadianOfDegree) * aLength) + sLastXPathFloat;
    if (aAddEntry) {
        sLastXPathFloat = tNewXPathFloat;
    }
    int tXDelta = int(tNewXPathFloat) - sLastXPathInt;
    *sXPathDeltaPtr = tXDelta;
    int tLastXPathInt = sLastXPathInt + tXDelta;
    if (aAddEntry) {
        sLastXPathInt = tLastXPathInt;
    }

    // X-Min and Max
    if (tLastXPathInt > sXPathMax) {
        sXPathMax = tLastXPathInt;
    } else if (tLastXPathInt < sXPathMin) {
        sXPathMin = tLastXPathInt;
    }

    // Y delta
    float tNewYPathFloat = (sin(tRadianOfDegree) * aLength) + sLastYPathFloat;
    if (aAddEntry) {
        sLastYPathFloat = tNewYPathFloat;
    }
    int tYDelta = int(tNewYPathFloat) - sLastYPathInt;
    *sYPathDeltaPtr = tYDelta;
    int tLastYPathInt = sLastYPathInt + tYDelta;
    if (aAddEntry) {
        sLastYPathInt = tLastYPathInt;
    }

    // Y-Min and Max
    if (tLastYPathInt > sYPathMax) {
        sYPathMax = tLastYPathInt;
    } else if (tLastYPathInt < sYPathMin) {
        sYPathMin = tLastYPathInt;
    }
    if (aAddEntry) {
        if (sXPathDeltaPtr < &xPathDelta[PATH_LENGTH_MAX - 2]) {
            sXPathDeltaPtr++;
            sYPathDeltaPtr++;
        }
    }
}

/*
 * Draw so that (forward) x direction is mapped to display y value since we have landscape layout
 * y+ is left y- is right
 */
void DrawPath() {
    BlueDisplay1.clearDisplay(COLOR_WHITE);

    BlueDisplay1.drawTextPGM(BUTTON_WIDTH_10_POS_4 - 8, TEXT_SIZE_22, PSTR("Robot Car Path"), TEXT_SIZE_16, COLOR_BLUE,
    COLOR_NO_BACKGROUND);
    TouchButtonBack.setPosition(BUTTON_WIDTH_4_POS_4, BUTTON_HEIGHT_4_LINE_4);
    TouchButtonBack.drawButton();
    TouchButtonResetPath.drawButton();
    TouchButtonStartStopAutonomousForPathPage.drawButton();

    /*
     * compute scale factor
     */
    int tXdelta = sXPathMax - sXPathMin;
    int tYdelta = sYPathMax - sYPathMin;
    uint8_t tScaleShift = 0;
    while (tXdelta > DISPLAY_HEIGHT || tYdelta >= DISPLAY_WIDTH) {
        tScaleShift++;
        tXdelta >>= 1;
        tYdelta >>= 1;
    }
//    BlueDisplay1.debug("ScaleShift=", tScaleShift);

    /*
     * Try to position start point at middle of bottom line
     */
    int tXDisplayPos;
    if (tYdelta < (DISPLAY_WIDTH / 2)) {
        tXDisplayPos = DISPLAY_WIDTH / 2;
    } else {
        // position at left so that sYPathMax (which is known to be > 0) fits on screen
        tXDisplayPos = (sYPathMax >> tScaleShift) + 2; // +2 for left border
    }
    int tYDisplayPos = DISPLAY_HEIGHT + (sXPathMin >> tScaleShift);

    /*
     * Draw Path -> map path x to display y
     */
    int * tXDeltaPtr = &xPathDelta[0];
    int * tYDeltaPtr = &yPathDelta[0];
    while (tXDeltaPtr <= sXPathDeltaPtr) {
        int tYDisplayDelta = (-(*tXDeltaPtr++)) >> tScaleShift;
        int tXDisplayDelta = (-(*tYDeltaPtr++)) >> tScaleShift;
        BlueDisplay1.drawLineRel(tXDisplayPos, tYDisplayPos, tXDisplayDelta, tYDisplayDelta, COLOR_RED);
        tXDisplayPos += tXDisplayDelta;
        tYDisplayPos += tYDisplayDelta;
    }
}

/*
 * Print VIN (used as motor supply) periodically
 */
void printVinPeriodically() {
    static uint32_t sMillisOfNextVCCInfo = 0;
    uint32_t tMillis = millis();

    if (tMillis >= sMillisOfNextVCCInfo) {
        sMillisOfNextVCCInfo = tMillis + 3000;
        char tDataBuffer[18];
        char tVCCString[6];
        // get value at A3 with 1.1 Volt reference
        float tVCC = getADCValue(VCC_11TH_IN_CHANNEL, INTERNAL);
        // assume resistor network of 100k / 10k (divider by 11)
        // tVCC * 0,01181640625
        float tVCCVoltage = tVCC * ((11.0 * 1.1) / 1024);
        dtostrf(tVCCVoltage, 4, 2, tVCCString);
        sprintf_P(tDataBuffer, PSTR("%s Volt"), tVCCString);
        BlueDisplay1.drawText(BUTTON_WIDTH_4_POS_4, BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND, tDataBuffer, TEXT_SIZE_11,
        COLOR_BLACK, COLOR_WHITE);
    }
}

void printMotorValues() {

    uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 25;
    sprintf_P(sStringBuffer, PSTR("min. %3d %3d"), myCar.leftMotorControl.MinSpeed, myCar.rightMotorControl.MinSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("max. %3d %3d"), myCar.leftMotorControl.MaxSpeed, myCar.rightMotorControl.MaxSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);

    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("comp. %2d  %2d"), myCar.leftMotorControl.SpeedCompensation,
            myCar.rightMotorControl.SpeedCompensation);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("act. %3d %3d"), myCar.leftMotorControl.ActualSpeed, myCar.rightMotorControl.ActualSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);

}

void printMotorDebugValues() {
    /*
     * Debug info
     */
    uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 70;
    sprintf_P(sStringBuffer, PSTR("ramp1%3d %3d"), myCar.leftMotorControl.DistanceCountAfterRampUp,
            myCar.rightMotorControl.DistanceCountAfterRampUp);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("endSp%3d %3d"), myCar.leftMotorControl.SpeedAtTargetCountReached,
            myCar.rightMotorControl.SpeedAtTargetCountReached);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("debug%3d %3d"), myCar.leftMotorControl.Debug, myCar.rightMotorControl.Debug);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("dcnt %3d %3d"), myCar.leftMotorControl.DebugCount, myCar.rightMotorControl.DebugCount);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);

    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("tcnt %3d %3d"), myCar.leftMotorControl.LastTargetDistanceCount,
            myCar.rightMotorControl.LastTargetDistanceCount);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}

void printDistanceValues() {
    uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 25 + 9 * TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("cnt %4d%4d%4d"), myCar.leftMotorControl.DistanceCount, myCar.rightMotorControl.DistanceCount,
            sCountPerScan);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}


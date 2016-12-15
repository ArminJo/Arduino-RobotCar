/*
 * RobotCarGui.cpp
 *
 *  Contains all the GUI elements for operating and controlling the RobotCar.
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
#include "ArminsUtils.h"

#include <avr/interrupt.h>
#include <AutonomousDrive.h>

/*
 * Motor GUI
 */
BDButton TouchButtonStartStop;
BDButton TouchButtonReset;
BDButton TouchButtonBack;
BDButton TouchButtonNextPage;
BDButton TouchButtonDirection;
BDButton TouchButtonStoreVelocity;

BDButton TouchButtonTest1;
BDButton TouchButtonTest2;
BDButton TouchButtonStepMode;
BDButton TouchButtonSingleStep;
BDButton TouchButtonSingleScan;

BDButton TouchButtonCalibrate;

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

bool sRunManual;
bool sRunTestSimple;
bool sRunTestPro;

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
void setTestSimpleButtonCaption();
void setTestProButtonCaption();

// a string buffer for any purpose...
char sDataBuffer[128];

void setupGUI(void) {
#ifdef USE_SIMPLE_SERIAL  // see line 38 in BlueSerial.h - use global #define USE_STANDARD_SERIAL to disable it
    initSimpleSerial(HC_05_BAUD_RATE, false);
#else
    Serial.begin(HC_05_BAUD_RATE);
#endif

    sActualPage = PAGE_MANUAL_CONTROL;

    // Register callback handler and check for connection
    BlueDisplay1.initCommunication(&initDisplay, &drawGui);
}

void loopGUI(void) {

    if (sActualPage != PAGE_SHOW_PATH) {

        // do not slow down ramps
        if (myCar.isState(MOTOR_STATE_STOPPED) || myCar.isState(MOTOR_STATE_FULL_SPEED)) {

            /*
             * Display changed values in GUI only at manual page
             */
            if (sActualPage == PAGE_MANUAL_CONTROL) {
                uint8_t ActualRequestedSpeed = myCar.rightMotorControl.ActualSpeed + myCar.rightMotorControl.SpeedCompensation;
                if (ActualRequestedSpeed != sLastSpeedSliderValue) {
                    SliderSpeed.setActualValueAndDrawBar(ActualRequestedSpeed);
                    sLastSpeedSliderValue = ActualRequestedSpeed;
                }
                if (EncoderMotorControl::ValuesHaveChanged) {
                    EncoderMotorControl::ValuesHaveChanged = false;
                    printMotorValues();
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

            /*
             * Print changed tick values
             */
            if (EncoderMotorControl::DistanceTickCounterHasChanged) {
                EncoderMotorControl::DistanceTickCounterHasChanged = false;
                printDistanceValues();
            }

            drawVinPeriodically();
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

void doCalibrate(BDButton * aTheTouchedButton, int16_t aValue) {
    EncoderMotorControl::calibrate();
}

void doTestSimple1(BDButton * aTheTouchedButton, int16_t aValue) {
    sRunTestSimple = !aValue;
    setTestSimpleButtonCaption();

    aTheTouchedButton->setValueAndDraw(sRunTestSimple);
    if (sRunTestSimple) {
        sRunTestPro = false;
        // disable other button
        setTestProButtonCaption();

        resetPathData();
        sStepMode = MODE_SINGLE_STEP;
        setStepModeButtonCaption();
        TouchButtonStepMode.drawButton();
        // enable motors
        myCar.activateMotors();
        sDegreeToTurn = 0;
    } else {
        // stop, but preserve direction
        myCar.shutdownMotors(false);
    }
}

void doTestPro(BDButton * aTheTouchedButton, int16_t aValue) {
    sRunTestPro = !aValue;

    setTestProButtonCaption();

    if (sRunTestPro) {
        // disable other button
        sRunTestSimple = false;
        setTestSimpleButtonCaption();

        resetPathData();
        // enable motors
        myCar.activateMotors();
        sDegreeToTurn = 0;
    } else {
        // stop, but preserve direction
        myCar.shutdownMotors(false);
    }
}

void rotate(int16_t aRotationDegrees, bool inPlace) {
    myCar.initRotateCar(aRotationDegrees, myCar.is2WDCar, inPlace);
}

void doRotation(BDButton * aTheTouchedButton, int16_t aValue) {
    rotate(aValue, myCar.isDirectionForward);
}

/*
 * Store ActualSpeed as MaxSpeed
 */
void doStoreMaxSpeed(BDButton * aTheTouchedButton, int16_t aValue) {
    if (sLastSpeedSliderValue != 0) {
        myCar.rightMotorControl.MaxSpeed = myCar.rightMotorControl.ActualSpeed;
        myCar.rightMotorControl.writeEeprom();

        // use the same value here !
        myCar.leftMotorControl.MaxSpeed = myCar.rightMotorControl.ActualSpeed;
        myCar.leftMotorControl.writeEeprom();
    }
}

void speedSliderSetValue(uint8_t aSpeed, bool aUpdateBar) {
    if (aUpdateBar) {
        SliderSpeed.setActualValueAndDrawBar(aSpeed);
    }
    sprintf_P(sDataBuffer, PSTR("%3d"), aSpeed);
    SliderSpeed.printValue(sDataBuffer);
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
    ServoWrite(sServoUS, aValue);
}

void setStartStopButtonCaption() {
    if (sRunManual) {
        // green stop button
        TouchButtonStartStop.setCaptionPGM(PSTR("Stop"));
    } else {
        // red start button
        TouchButtonStartStop.setCaptionPGM(PSTR("Start"));
    }
    TouchButtonStartStop.setValueAndDraw(sRunManual);
}

void setTestSimpleButtonCaption() {
    if (sRunTestSimple) {
        // green stop button
        TouchButtonTest1.setCaptionPGM(PSTR("Stop\nSimple"));
    } else {
        // red start button
        TouchButtonTest1.setCaptionPGM(PSTR("Start\nSimple"));
    }
    TouchButtonTest1.setValueAndDraw(sRunTestSimple);
}

void setTestProButtonCaption() {
    if (sRunTestPro) {
        // green stop button
        TouchButtonTest2.setCaptionPGM(PSTR("Stop\nPro"));
    } else {
        // red start button
        TouchButtonTest2.setCaptionPGM(PSTR("Start\nPro"));
    }
    TouchButtonTest2.setValueAndDraw(sRunTestPro);
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
void doStartStop(BDButton * aTheTouchedButton, int16_t aValue) {
    sRunManual = !sRunManual;
    if (sRunManual) {
        // enable motors
        myCar.activateMotors();
        setStartStopButtonCaption();
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
    if (sRunTestSimple) {
        tInfoWasProcessed = fillForwardDistancesInfoSimple(&ForwardDistancesInfo, sServoUS, true, true);
    } else {
        clearPrintedForwardDistancesInfos();
        tInfoWasProcessed = fillForwardDistancesInfoPro(&ForwardDistancesInfo, sServoUS, true, true);
        doWallDetectionPro(&ForwardDistancesInfo);
        sDegreeToTurn = doCollisionDetectionPro(&ForwardDistancesInfo);
        printCollisionDecision(sDegreeToTurn, CENTIMETER_PER_RIDE_PRO, false);
    }
    if (!tInfoWasProcessed) {
        drawForwardDistancesInfos(&ForwardDistancesInfo);
    }
}

void setDirectionButtonCaption() {
    if (myCar.isDirectionForward) {
// direction forward
        TouchButtonDirection.setCaption("\x88");
    } else {
// direction backward
        TouchButtonDirection.setCaption("\x87");
    }
}

/*
 * Stops motors and change direction
 */
void GUIchangeDirection(BDButton * aTheTouchedButton, int16_t aValue) {
    myCar.setDirection(!aValue);

    sRunManual = false;
    setDirectionButtonCaption();
    TouchButtonDirection.drawButton();
    setStartStopButtonCaption();
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
     * US Sliders
     */

    SliderUSDistance.init(BUTTON_WIDTH_6_POS_6 - BUTTON_WIDTH_10 - 4, 10, BUTTON_WIDTH_10, US_SLIDER_SIZE, 200, 0,
    SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT,
    NULL);
    SliderUSDistance.setScaleFactor(2);
    SliderUSDistance.setValueUnitString("cm");

    SliderUSPosition.init(BUTTON_WIDTH_6_POS_6, 10, BUTTON_WIDTH_6, US_SLIDER_SIZE, US_SLIDER_SIZE / 2, US_SLIDER_SIZE / 2,
    COLOR_YELLOW, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doUSPosition);
    SliderUSPosition.setScaleFactor(180.0 / US_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderUSPosition.setValueUnitString("\xB0");

    /*
     * Control buttons
     */
    TouchButtonBack.initPGM(BUTTON_WIDTH_4_POS_4, 0, BUTTON_WIDTH_4, BUTTON_HEIGHT_4, COLOR_RED, PSTR("Back"), TEXT_SIZE_22,
            BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doBack);

    TouchButtonStartStop.init(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_BLUE, "", TEXT_SIZE_22,
            BUTTON_FLAG_DO_BEEP_ON_TOUCH | BUTTON_FLAG_TYPE_AUTO_RED_GREEN, sRunManual, &doStartStop);
    setStartStopButtonCaption();

    TouchButtonReset.initPGM(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_BLUE, PSTR("Reset"), TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doReset);

    TouchButtonNextPage.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, "",
    TEXT_SIZE_16, BUTTON_FLAG_DO_BEEP_ON_TOUCH, myCar.isDirectionForward, &GUISwitchPages);
    setSwitchPagesButtonCaption();

    TouchButtonStoreVelocity.initPGM(0, BUTTON_HEIGHT_4_LINE_4 - BUTTON_HEIGHT_8_LINE_2 + 1, BUTTON_WIDTH_6, BUTTON_HEIGHT_8,
    COLOR_BLUE, PSTR("Store"), TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doStoreMaxSpeed);

    TouchButtonStepMode.init(0, 0, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_4, COLOR_BLUE, "", TEXT_SIZE_18, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0,
            &doStepMode);
    setStepModeButtonCaption();

    TouchButtonSingleScan.initPGM(0, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_4, COLOR_BLUE, PSTR("Scan"),
    TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doSingleScan);

    TouchButtonSingleStep.initPGM(0, BUTTON_HEIGHT_4_LINE_3, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_4, COLOR_BLUE, PSTR("Step"),
    TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 0, &doSingleStep);

    TouchButtonTest1.init(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, "",
    TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH | BUTTON_FLAG_TYPE_AUTO_RED_GREEN, sRunTestSimple, &doTestSimple1);
    setTestSimpleButtonCaption();

    TouchButtonTest2.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, "",
    TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH | BUTTON_FLAG_TYPE_AUTO_RED_GREEN, sRunTestPro, &doTestPro);
    setTestProButtonCaption();

    /*
     * Test buttons
     */
    TouchButton5cm.initPGM(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, PSTR("5cm"),
    TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 5, &doDistance);
    TouchButton10cm.initPGM(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, PSTR("10cm"),
    TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 10, &doDistance);

    TouchButton20cm.initPGM(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, PSTR("20cm"),
    TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 20, &doDistance);
    TouchButton40cm.initPGM(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, PSTR("40cm"),
    TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 40, &doDistance);

    TouchButton45DegreeLeft.initPGM(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("45\xB0"),
            TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 45, &doRotation);
    TouchButton45DegreeRight.initPGM(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("45\xB0"),
            TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, -45, &doRotation);

    TouchButton90DegreeLeft.initPGM(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("90\xB0"),
            TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 90, &doRotation);
    TouchButton90DegreeRight.initPGM(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("90\xB0"),
            TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, -90, &doRotation);
    TouchButton360Degree.initPGM(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            PSTR("360\xB0"),
            TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 360, &doRotation);

    TouchButtonDirection.initPGM(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, "",
    TEXT_SIZE_22, BUTTON_FLAG_DO_BEEP_ON_TOUCH, myCar.isDirectionForward, &GUIchangeDirection);
    setDirectionButtonCaption();

    TouchButtonCalibrate.initPGM(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_RED,
            PSTR("cal"),
            TEXT_SIZE_11, BUTTON_FLAG_DO_BEEP_ON_TOUCH, 1, &doCalibrate);
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
        TouchButtonStartStop.drawButton();
        TouchButtonReset.drawButton();

        TouchButtonDirection.drawButton();
        TouchButtonCalibrate.drawButton();

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
        TouchButtonStoreVelocity.drawButton();

        SliderUSPosition.drawSlider();
        SliderUSDistance.drawSlider();

        printMotorValues();
        printDistanceValues();

    } else if (sActualPage == PAGE_AUTOMATIC_CONTROL) {

        TouchButtonBack.drawButton();

        TouchButtonTest1.drawButton();
        TouchButtonTest2.drawButton();
        TouchButtonStepMode.drawButton();
        TouchButtonSingleScan.drawButton();
        TouchButtonSingleStep.drawButton();
        drawForwardDistancesInfos(&ForwardDistancesInfo);
        printCollisionDecision(sDegreeToTurn, CENTIMETER_PER_RIDE_PRO, false);

    } else {
        DrawPath();
    }
}

void resetGUIControls() {
    myCar.resetAndShutdownMotors();

    sRunTestSimple = false;
    sRunTestPro = false;

    if (sActualPage == PAGE_AUTOMATIC_CONTROL) {
        TouchButtonTest1.setValueAndDraw(sRunTestSimple);
        TouchButtonTest2.setValueAndDraw(sRunTestPro);
    } else {
        TouchButtonTest1.setValue(sRunTestSimple);
        TouchButtonTest2.setValue(sRunTestPro);
    }

    sRunManual = false;
    sDoStep = false;
    sStepMode = MODE_CONTINUOUS;

    setDirectionButtonCaption();
    setStartStopButtonCaption();
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
            unsigned int tCentimeter = getUSDistanceAsCentiMeterWithCentimeterTimeout(256);
            // feedback as slider length
            if (tCentimeter != sLastCentimeterToObstacle) {
                sLastCentimeterToObstacle = tCentimeter;
                showDistance(tCentimeter);
            }
        }
    }
}

void printSingleDistanceVector(uint16_t aLength, int aDegree, Color_t aColor) {
    BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, aLength, aDegree, aColor, 3);
}

void printCollisionDecision(int aDegreeToTurn, uint8_t aLengthOfVector, bool aDoClear) {
    if (sActualPage == PAGE_AUTOMATIC_CONTROL) {
        Color_t tColor = COLOR_CYAN;
        int tDegreeToDisplay = aDegreeToTurn;
        if (tDegreeToDisplay == 180) {
            tColor = COLOR_MAGENTA;
            tDegreeToDisplay = 0;
        }
        if (aDoClear) {
            tColor = COLOR_WHITE;
        }
        BlueDisplay1.drawVectorDegree(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, aLengthOfVector, tDegreeToDisplay + 90,
                tColor);
        if (!aDoClear) {
            sprintf_P(sDataBuffer, PSTR("rotation: %3d\xB0"), aDegreeToTurn);
            BlueDisplay1.drawText(US_DISTANCE_MAP_ORIGIN_X - (7 * TEXT_SIZE_11_WIDTH), US_DISTANCE_MAP_ORIGIN_Y + TEXT_SIZE_11,
                    sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
        }
    }
}

int8_t xPathDelta[PATH_LENGTH_MAX], yPathDelta[PATH_LENGTH_MAX];
int8_t *sXPathDeltaPtr, *sYPathDeltaPtr;
// exact float values to avoid aggregating rounding errors
float sLastXPathFloat, sLastYPathFloat;
int sLastXPathInt, sLastYPathInt;
int sXPathMax, sXPathMin, sYPathMax, sYPathMin;
int sLastPathDirectionDegree;

void resetPathData() {
    sXPathDeltaPtr = &xPathDelta[0];
    sYPathDeltaPtr = &yPathDelta[0];
// set to origin
    sLastXPathFloat = 0.0;
    sLastYPathFloat = 0.0;
    sXPathMax = sYPathMax = 0;
    sXPathMin = sYPathMin = 0;

    sLastPathDirectionDegree = 0;
// clear delta arrays
    for (unsigned int i = 0; i < sizeof(xPathDelta); ++i) {
        xPathDelta[i] = 0;
        yPathDelta[i] = 0;
    }
}

/*
 * (Over-)writes to actual pointer position
 */
void insertToPath(int aDegree, int aLengthCentimeter) {
    sLastPathDirectionDegree += aDegree;
    float tRadianOfDegree = sLastPathDirectionDegree * (M_PI / 180);
    /*
     * compute X and Y delta and min/max
     */
    float tNewXPathFloat = (cos(tRadianOfDegree) * aLengthCentimeter) + sLastXPathFloat;
    sLastXPathFloat = tNewXPathFloat;
    int tXDelta = int(tNewXPathFloat) - sLastXPathInt;
    *sXPathDeltaPtr = tXDelta;
    int tLastXPathInt = sLastXPathInt + tXDelta;
    sLastXPathInt = tLastXPathInt;
    // Min - Max
    if (tLastXPathInt > sXPathMax) {
        sXPathMax = tLastXPathInt;
    } else if (tLastXPathInt < sXPathMin) {
        sXPathMin = tLastXPathInt;
    }

    // Y delta
    float tNewYPathFloat = (sin(tRadianOfDegree) * aLengthCentimeter) + sLastYPathFloat;
    sLastYPathFloat = tNewYPathFloat;
    int tYDelta = int(tNewYPathFloat) - sLastYPathInt;
    *sYPathDeltaPtr = tYDelta;
    int tLastYPathInt = sLastYPathInt + tYDelta;
    sLastYPathInt = tLastYPathInt;
    // Min - Max
    if (tLastYPathInt > sYPathMax) {
        sYPathMax = tLastYPathInt;
    } else if (tLastYPathInt < sYPathMin) {
        sYPathMin = tLastYPathInt;
    }
}

/*
 * Adds an new set of deltas to the Path array and updates min and max values
 */
void addToPath(int aLengthCentimeter, int aDegree) {
    insertToPath(aDegree, aLengthCentimeter);
    // avoid overflow
    if (sXPathDeltaPtr < &xPathDelta[sizeof(xPathDelta) - 2]) {
        sXPathDeltaPtr++;
        sYPathDeltaPtr++;
    }
}

void DrawPath() {
    BlueDisplay1.clearDisplay(COLOR_WHITE);

    BlueDisplay1.drawTextPGM(BUTTON_WIDTH_10_POS_4 - 8, TEXT_SIZE_22, PSTR("Robot Car Path"), TEXT_SIZE_16, COLOR_BLUE,
    COLOR_NO_BACKGROUND);
    TouchButtonBack.drawButton();

    /*
     * compute scale factor
     */
    int tXdelta = sXPathMax - sXPathMin;
    int tYdelta = sYPathMax - sYPathMin;
    uint8_t tScaleShift = 0;
    while (tXdelta > DISPLAY_WIDTH || tYdelta >= DISPLAY_HEIGHT) {
        tScaleShift++;
        tXdelta >>= 1;
        tYdelta >>= 1;
    }

    /*
     * Try to position Y at middle of screen
     */
    int tYPos;
    if (tYdelta < (DISPLAY_HEIGHT / 2)) {
        tYPos = DISPLAY_HEIGHT / 2;
    } else {
        // position at bottom -2 for border
        tYPos = (DISPLAY_HEIGHT - 2) + (sYPathMin >> tScaleShift);
    }
    int tXPos = -(sXPathMin >> tScaleShift);

    /*
     * Draw Path
     */
    int8_t * tXptr = &xPathDelta[0];
    int8_t * tYptr = &yPathDelta[0];
    while (tXptr < sXPathDeltaPtr) {
        int8_t tXDelta = (*tXptr++) >> tScaleShift;
        int8_t tYDelta = -((*tYptr++) >> tScaleShift);
        BlueDisplay1.drawLineRel(tXPos, tYPos, tXDelta, tYDelta, COLOR_RED);
        tXPos += tXDelta;
        tYPos += tYDelta;
    }
}

/*
 * Print VIN (used as motor supply) periodically
 */
void drawVinPeriodically() {
    static uint32_t sMillisOfNextVCCInfo = 0;
    uint32_t tMillis = millis();

    if (tMillis >= sMillisOfNextVCCInfo) {
        sMillisOfNextVCCInfo = tMillis + 3000;
        char tDataBuffer[18];
        char tVCCString[6];
        // get value at A3 with 1.1 Volt reference
        float tVCC = getADCValue(3, INTERNAL);
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
    sprintf_P(sDataBuffer, PSTR("min. %3d %3d"), myCar.leftMotorControl.MinSpeed, myCar.rightMotorControl.MinSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sDataBuffer, PSTR("max. %3d %3d"), myCar.leftMotorControl.MaxSpeed, myCar.rightMotorControl.MaxSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sDataBuffer, PSTR("act. %3d %3d"), myCar.leftMotorControl.ActualSpeed, myCar.rightMotorControl.ActualSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sDataBuffer, PSTR("comp. %2d  %2d"), myCar.leftMotorControl.SpeedCompensation,
            myCar.rightMotorControl.SpeedCompensation);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sDataBuffer, PSTR("ramp1%3d %3d"), myCar.leftMotorControl.DistanceCountAfterRampUp,
            myCar.rightMotorControl.DistanceCountAfterRampUp);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    /*
     * Debug info
     */
    tYPos += TEXT_SIZE_11;
    sprintf_P(sDataBuffer, PSTR("endSp%3d %3d"), myCar.leftMotorControl.SpeedAtTargetCountReached,
            myCar.rightMotorControl.SpeedAtTargetCountReached);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sDataBuffer, PSTR("debug%3d %3d"), myCar.leftMotorControl.Debug, myCar.rightMotorControl.Debug);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sDataBuffer, PSTR("dcnt %3d %3d"), myCar.leftMotorControl.DebugCount, myCar.rightMotorControl.DebugCount);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);

    tYPos += TEXT_SIZE_11;
    sprintf_P(sDataBuffer, PSTR("tcnt %3d %3d"), myCar.leftMotorControl.LastTargetDistanceCount,
            myCar.rightMotorControl.LastTargetDistanceCount);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}

void printDistanceValues() {
    uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 25 + 9 * TEXT_SIZE_11;
    sprintf_P(sDataBuffer, PSTR("cnt %4d%4d%4d"), myCar.leftMotorControl.DistanceCount, myCar.rightMotorControl.DistanceCount,
            sCountPerScan);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}


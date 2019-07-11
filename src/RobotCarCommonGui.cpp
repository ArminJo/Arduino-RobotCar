/*
 * RobotCarCommonGui.cpp
 *
 *  Contains all common GUI elements for operating and controlling the RobotCar.
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

uint8_t sActualPage;
BDButton TouchButtonBackSmall;
BDButton TouchButtonBack;
BDButton TouchButtonNextPage;
BDButton TouchButtonCalibrate;

bool sStarted; // main start flag. If true motors are running
BDButton TouchButtonRobotCarStartStop;
BDButton TouchButtonDirection;

BDSlider SliderSpeed;
uint16_t sLastSpeedSliderValue = 0;

BDSlider SliderSpeedRight;
BDSlider SliderSpeedLeft;

// a string buffer for any purpose...
char sStringBuffer[128];

void setupGUI(void) {
#ifdef USE_SIMPLE_SERIAL  // see line 38 in BlueSerial.h - use global #define USE_STANDARD_SERIAL to disable it
    initSimpleSerial(HC_05_BAUD_RATE);
#else
    Serial.begin(HC_05_BAUD_RATE);
#endif

    sActualPage = PAGE_HOME;

    // Register callback handler and check for connection
    // This leads to call to initDisplay() and drawHomePage() after connect
    BlueDisplay1.initCommunication(&initDisplay, &startHomePage);
}

void delayAndLoopGUI(uint16_t aDelayMillis) {
    uint32_t tStartMillis = millis();
    do {
        loopGUI();
    } while (millis() - tStartMillis > aDelayMillis);
}

void loopGUI(void) {

    // do not show anything during motor speed ramps
    if (RobotCar.isState(MOTOR_STATE_STOPPED) || RobotCar.isState(MOTOR_STATE_FULL_SPEED)) {

        /*
         * Display changed values in GUI only at manual page
         */

        if (sActualPage == PAGE_HOME) {
            loopHomePage();
        } else if (sActualPage == PAGE_TEST) {
            loopTestPage();
        } else if (sActualPage == PAGE_AUTOMATIC_CONTROL) {
            loopAutonomousDrivePage();
        } else if (sActualPage == PAGE_SHOW_PATH) {
            loopPathInfoPage();
        }

        // for all but PathInfo page
        if (sActualPage != PAGE_SHOW_PATH) {
            /*
             * Print changed tick values
             */
            if (EncoderMotor::DistanceTickCounterHasChanged) {
                EncoderMotor::DistanceTickCounterHasChanged = false;
                printDistanceValues();
            }

            readAndPrintVinPeriodically();
        }
    }

    /*
     * Check if receive buffer contains an event
     * Do it at end since it may change the condition for calling this function and its printing
     */
    checkAndHandleEvents();
}

void setStartStopButtonValue() {
    TouchButtonRobotCarStartStop.setValueAndDraw(sStarted);
}

/*
 * Handle Start/Stop
 */
void startStopRobotCar(bool aNewStartedValue) {
    sStarted = aNewStartedValue;
    TouchButtonRobotCarStartStop.setValue(aNewStartedValue, (sActualPage == PAGE_HOME));

    if (sStarted) {
        // enable motors
        RobotCar.activateMotors();
    } else {
        // stop, but preserve direction
        RobotCar.shutdownMotors(false);
        if (sActualPage == PAGE_HOME|| sActualPage == PAGE_TEST) {
            SliderSpeed.setActualValueAndDrawBar(0);
        }
    }
}

void doRobotCarStartStop(BDButton * aTheTouchedButton, int16_t aValue) {
    startStopRobotCar(!sStarted);
}

void doCalibrate(BDButton * aTheTouchedButton, int16_t aValue) {
    EncoderMotor::calibrate();
}

/*
 * Minimum Speed is 30 for USB power and no load, 50 for load
 * Minimum Speed is 20 for 2 Lithium 18650 battery power and no load, 25 for load
 */
void doSpeedSlider(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    if (aValue != sLastSpeedSliderValue) {
        if (!sStarted) {
            startStopRobotCar(true);
        }
        sLastSpeedSliderValue = aValue;
        RobotCar.setSpeedCompensated(aValue);
    }
}

/*
 * Updates and shows speed slider value according to ActualSpeed
 */
void showSpeedSliderValue() {
    uint8_t tActualRequestedSpeed;
    if (rightEncoderMotor.SpeedCompensation == 0) {
        tActualRequestedSpeed = rightEncoderMotor.ActualSpeed;
    } else {
        tActualRequestedSpeed = leftEncoderMotor.ActualSpeed;
    }
    if (tActualRequestedSpeed != sLastSpeedSliderValue) {
        SliderSpeed.setActualValueAndDrawBar(tActualRequestedSpeed);
        sLastSpeedSliderValue = tActualRequestedSpeed;
    }
}

/*
 * Display velocity as slider values
 */
void displayVelocitySliderValues() {
    EncoderMotor * tMotorInfo = &leftEncoderMotor;
    BDSlider * tSliderPtr = &SliderSpeedLeft;
    uint16_t tXPos = 0;
    for (int i = 0; i < 2; ++i) {
        if (EncoderMotor::DistanceTickCounterHasChanged) {
            tSliderPtr->setActualValueAndDrawBar(tMotorInfo->ActualVelocity);
        }
        tMotorInfo = &rightEncoderMotor;
        tSliderPtr = &SliderSpeedRight;
        tXPos += BUTTON_WIDTH_16 + 4;
    }
}
/*
 * Stops motors and change direction
 */
void doChangeDirection(BDButton * aTheTouchedButton, int16_t aValue) {
    RobotCar.setDirection(!RobotCar.isDirectionForward);
    setDirectionButtonCaption();
    TouchButtonDirection.drawButton();
    startStopRobotCar(false);
}

void setDirectionButtonCaption() {
    if (RobotCar.isDirectionForward) {
// direction forward
        TouchButtonDirection.setCaption("\x87");
    } else {
// direction backward
        TouchButtonDirection.setCaption("\x88");
    }
}

/*
 * For Next and Back button
 * Stop old page and start new one
 */
void GUISwitchPages(BDButton * aTheTouchedButton, int16_t aValue) {
    /*
     * Stop old page
     */
    if (sActualPage == PAGE_HOME) {
        stopHomePage();
    } else if (sActualPage == PAGE_AUTOMATIC_CONTROL) {
        stopAutonomousDrivePage();
    } else if (sActualPage == PAGE_SHOW_PATH) {
        stopPathInfoPage();
    } else if (sActualPage == PAGE_TEST) {
        stopTestPage();
        aValue = -3; // only back to home permitted
    }

    /*
     * determine next page
     */
    sActualPage += aValue;
    // handle overflow and unsigned underflow
    if (sActualPage > PAGE_LAST_NUMBER) {
        sActualPage = PAGE_HOME;
    }

//    BlueDisplay1.debug("Actual page=", sActualPage);

    /*
     * Start new page
     */
    if (sActualPage == PAGE_HOME) {
        startHomePage();
    } else if (sActualPage == PAGE_AUTOMATIC_CONTROL) {
        startAutonomousDrivePage();
    } else if (sActualPage == PAGE_SHOW_PATH) {
        startPathInfoPage();
    } else if (sActualPage == PAGE_TEST) {
        startTestPage();
    }
}

void initDisplay(void) {
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_TOUCH_BASIC_DISABLE | BD_FLAG_USE_MAX_SIZE, DISPLAY_WIDTH,
    DISPLAY_HEIGHT);
    BlueDisplay1.setCharacterMapping(0x87, 0x2227); // mapping for AND - Forward
    BlueDisplay1.setCharacterMapping(0x88, 0x2228); // mapping for OR - Backwards
    // Since landscape has 2 orientations, let the user choose the right one.
    BlueDisplay1.setScreenOrientationLock(FLAG_SCREEN_ORIENTATION_LOCK_ACTUAL);

    /*
     * Common control buttons
     */
    TouchButtonRobotCarStartStop.init(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_BLUE, F("Start"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sStarted, &doRobotCarStartStop);
    TouchButtonRobotCarStartStop.setCaptionForValueTrue(F("Stop"));

    TouchButtonNextPage.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, "",
            TEXT_SIZE_16, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &GUISwitchPages);

    TouchButtonBack.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, F("Back"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, -1, &GUISwitchPages);

    TouchButtonBackSmall.init(BUTTON_WIDTH_4_POS_4, 0, BUTTON_WIDTH_4, BUTTON_HEIGHT_4, COLOR_RED, F("Back"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, -1, &GUISwitchPages);

    TouchButtonCalibrate.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_RED, F("CAL"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doCalibrate);

    // Direction
    TouchButtonDirection.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, "",
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doChangeDirection);
    setDirectionButtonCaption();

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

    initHomePage();
    initTestPage();
    initAutonomousDrivePage();
    initPathInfoPage();
}

void drawCommonGui(void) {
    BlueDisplay1.clearDisplay();

    BlueDisplay1.deactivateAllButtons();
    BlueDisplay1.deactivateAllSliders();

    BlueDisplay1.drawText(BUTTON_WIDTH_10_POS_4, TEXT_SIZE_22_HEIGHT, F("Robot Car"), TEXT_SIZE_22, COLOR_BLUE,
    COLOR_NO_BACKGROUND);

}

/*
 * Print VIN (used as motor supply) periodically
 * returns true if voltage was printed
 */
void readAndPrintVinPeriodically() {
    static uint32_t sMillisOfNextVCCInfo = 0;
    uint32_t tMillis = millis();

    if (tMillis >= sMillisOfNextVCCInfo) {
        sMillisOfNextVCCInfo = tMillis + PRINT_VOLTAGE_PERIOD_MILLIS;
        char tDataBuffer[18];
        char tVCCString[6];
        readVINVoltage();
        dtostrf(sVINVoltage, 4, 2, tVCCString);
        sprintf_P(tDataBuffer, PSTR("%s Volt"), tVCCString);
        if (sActualPage == PAGE_HOME) {
            BlueDisplay1.drawText(BUTTON_WIDTH_8_POS_4,
            BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER) - TEXT_SIZE_11_DECEND, tDataBuffer,
            TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
        } else {
            BlueDisplay1.drawText(BUTTON_WIDTH_4_POS_4, BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND, tDataBuffer, TEXT_SIZE_11,
            COLOR_BLACK, COLOR_WHITE);
        }
    }
}

void printMotorValues() {

    uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 25 + TEXT_SIZE_11_HEIGHT;
    sprintf_P(sStringBuffer, PSTR("min. %3d %3d"), leftEncoderMotor.MinSpeed, rightEncoderMotor.MinSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("max. %3d %3d"), leftEncoderMotor.MaxSpeed, rightEncoderMotor.MaxSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);

    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("comp. %2d  %2d"), leftEncoderMotor.SpeedCompensation, rightEncoderMotor.SpeedCompensation);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("act. %3d %3d"), leftEncoderMotor.ActualSpeed, rightEncoderMotor.ActualSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);

}

void printDistanceValues() {
    uint16_t tYPos;
    if (sActualPage == PAGE_AUTOMATIC_CONTROL) {
        tYPos = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;
    } else {
        tYPos = SPEED_SLIDER_SIZE / 2 + 25;
    }
    sprintf_P(sStringBuffer, PSTR("%4d %4d%3d"), leftEncoderMotor.DistanceCount, rightEncoderMotor.DistanceCount, sCountPerScan);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}

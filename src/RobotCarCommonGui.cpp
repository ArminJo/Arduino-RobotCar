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

#include "RobotCar.h"
#include "RobotCarGui.h"

// a string buffer for BD info output
char sStringBuffer[128];

uint8_t sCurrentPage;
BDButton TouchButtonBackSmall;
BDButton TouchButtonBack;
BDButton TouchButtonNextPage;
BDButton TouchButtonCalibrate;

bool sRobotCarStarted; // main start flag. If true motors are running
BDButton TouchButtonRobotCarStartStop;
BDButton TouchButtonDirection;

BDSlider SliderSpeed;
uint16_t sLastSpeedSliderValue = 0;

BDSlider SliderSpeedRight;
BDSlider SliderSpeedLeft;

/*
 * UltraSonic control GUI
 */
BDSlider SliderUSPosition;
BDSlider SliderUSDistance;
unsigned int sSliderUSLastCentimeter;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
BDSlider SliderIRDistance;
unsigned int sSliderIRLastCentimeter;
#endif

uint32_t sMillisOfNextVCCInfo = 0;

void setupGUI(void) {
    initSerial(BLUETOOTH_BAUD_RATE);

    sCurrentPage = PAGE_HOME;

    // Register callback handler and check for connection
    // This leads to call to initDisplay() and startCurrentPage() after connect
    BlueDisplay1.initCommunication(&initDisplay, &startCurrentPage);
}

void delayAndLoopGUI(uint16_t aDelayMillis) {
    uint32_t tStartMillis = millis();
    do {
        loopGUI();
    } while (millis() - tStartMillis <= aDelayMillis);
}

void loopGUI(void) {

    // do not show anything during motor speed ramps
    if (RobotCar.isState(MOTOR_STATE_STOPPED) || RobotCar.isState(MOTOR_STATE_FULL_SPEED)) {

        /*
         * Display changed values in GUI only at manual page
         */

        if (sCurrentPage == PAGE_HOME) {
            loopHomePage();
        } else if (sCurrentPage == PAGE_TEST) {
            loopTestPage();
        } else if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
            loopAutonomousDrivePage();
        } else if (sCurrentPage == PAGE_SHOW_PATH) {
            loopPathInfoPage();
        }

        // for all but PathInfo page
        if (sCurrentPage != PAGE_SHOW_PATH) {
            if (sCurrentPage != PAGE_AUTOMATIC_CONTROL) {
                /*
                 * Print changed tick values
                 */
                if (EncoderMotor::DistanceTickCounterHasChanged) {
                    EncoderMotor::DistanceTickCounterHasChanged = false;
                    printDistanceValues();
                }
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
    TouchButtonRobotCarStartStop.setValueAndDraw(sRobotCarStarted);
}

/*
 * Updates and shows speed slider value according to ActualSpeed
 */
void showSpeedSliderValue() {
    if (rightEncoderMotor.ActualSpeed != sLastSpeedSliderValue) {
        SliderSpeed.setValueAndDrawBar(rightEncoderMotor.ActualSpeed);
        sLastSpeedSliderValue = rightEncoderMotor.ActualSpeed;
    }
}

/*
 * Handle Start/Stop
 */
void startStopRobotCar(bool aNewStartedValue) {
    sRobotCarStarted = aNewStartedValue;
    TouchButtonRobotCarStartStop.setValue(aNewStartedValue, (sCurrentPage == PAGE_HOME));

    if (sRobotCarStarted) {
        // enable motors
        RobotCar.activateMotors();
    } else {
        // stop, but preserve direction
        RobotCar.shutdownMotors(false);
    }

    if (sCurrentPage == PAGE_HOME || sCurrentPage == PAGE_TEST) {
        showSpeedSliderValue();
    }
}

void doRobotCarStartStop(BDButton * aTheTouchedButton, int16_t aValue) {
    startStopRobotCar(!sRobotCarStarted);
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
        if (!sRobotCarStarted) {
            startStopRobotCar(true);
        }
        sLastSpeedSliderValue = aValue;
        RobotCar.setSpeedCompensated(aValue);
    }
}

void doUSServoPosition(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    DistanceServoWriteAndDelay(aValue);
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

void startCurrentPage() {
    switch (sCurrentPage) {
    case PAGE_HOME:
        startHomePage();
        break;
    case PAGE_AUTOMATIC_CONTROL:
        startAutonomousDrivePage();
        break;
    case PAGE_SHOW_PATH:
        startPathInfoPage();
        break;
    case PAGE_TEST:
        startTestPage();
        break;
    }
}
/*
 * For Next and Back button
 * Stop old page and start new one
 * @param aValue then difference between the current and the new page number. sCurrentPage += aValue;
 */
void GUISwitchPages(BDButton * aTheTouchedButton, int16_t aValue) {

    /*
     * Stop old page
     */
    switch (sCurrentPage) {
    case PAGE_HOME:
        stopHomePage();
        break;
    case PAGE_AUTOMATIC_CONTROL:
        stopAutonomousDrivePage();
        break;
    case PAGE_SHOW_PATH:
        stopPathInfoPage();
        break;
    case PAGE_TEST:
        stopTestPage();
        aValue = -3; // only back to home permitted
        break;
    }

    /*
     * determine next page
     */
    sCurrentPage += aValue;
    // handle overflow and unsigned underflow
    if (sCurrentPage > PAGE_LAST_NUMBER) {
        sCurrentPage = PAGE_HOME;
    }

//    BlueDisplay1.debug("Actual page=", sCurrentPage);

    /*
     * Start new page
     */
    startCurrentPage();
}

void initDisplay(void) {
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_TOUCH_BASIC_DISABLE | BD_FLAG_USE_MAX_SIZE, DISPLAY_WIDTH,
    DISPLAY_HEIGHT);
    BlueDisplay1.setCharacterMapping(0x87, 0x2227); // mapping for AND - Forward
    BlueDisplay1.setCharacterMapping(0x88, 0x2228); // mapping for OR - Backwards
    // Lock to landscape layout
    BlueDisplay1.setScreenOrientationLock(FLAG_SCREEN_ORIENTATION_LOCK_SENSOR_LANDSCAPE);

    /*
     * Common control buttons
     */
    TouchButtonRobotCarStartStop.init(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_BLUE, F("Start"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sRobotCarStarted, &doRobotCarStartStop);
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
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, RobotCar.isDirectionForward, &doChangeDirection);
    setDirectionButtonCaption();

    /*
     * Speed Sliders
     */
    SliderSpeed.init(0, 10, BUTTON_WIDTH_6, SPEED_SLIDER_SIZE, 200, 0, COLOR_YELLOW, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE, &doSpeedSlider);
    SliderSpeed.setScaleFactor(255.0 / SPEED_SLIDER_SIZE); // Slider is virtually 2 times larger than displayed, values were divided by 2

    SliderSpeedLeft.init(BUTTON_WIDTH_6 + 4, 0, BUTTON_WIDTH_16, SPEED_SLIDER_SIZE / 2, SPEED_SLIDER_SIZE / 2 - 1, 0,
    SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderSpeedLeft.setValueFormatString("%3d"); // Since we also send values grater 100

    SliderSpeedRight.init(BUTTON_WIDTH_6 + 4 + BUTTON_WIDTH_16 + 8, 0, BUTTON_WIDTH_16, SPEED_SLIDER_SIZE / 2,
    SPEED_SLIDER_SIZE / 2 - 1, 0, SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderSpeedRight.setValueFormatString("%3d");

    /*
     * scaled (0 to 180) US Sliders
     */
    SliderUSPosition.init(BUTTON_WIDTH_6_POS_6, 10, BUTTON_WIDTH_6, US_SLIDER_SIZE, 90, 90, COLOR_YELLOW, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE, &doUSServoPosition);
    SliderUSPosition.setBarThresholdColor(COLOR_BLUE);
    SliderUSPosition.setScaleFactor(180.0 / US_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderUSPosition.setValueUnitString("\xB0"); // \xB0 is degree character

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    /*
     * Two thin sliders with captions and without cm units
     */
    SliderIRDistance.init((BUTTON_WIDTH_6_POS_6 - BUTTON_WIDTH_10) - 4, 10, (BUTTON_WIDTH_10 / 2) - 2, US_SLIDER_SIZE,
    DISTANCE_TIMEOUT_CM, 0, SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderIRDistance.setScaleFactor(2); // Slider is virtually 2 times larger, values were divided by 2
    SliderIRDistance.setBarThresholdColor(DISTANCE_TIMEOUT_COLOR);
    SliderIRDistance.setCaptionProperties(10, FLAG_SLIDER_CAPTION_ALIGN_RIGHT | FLAG_SLIDER_CAPTION_BELOW, 2, COLOR_BLACK,
    COLOR_WHITE);
    SliderIRDistance.setCaption("IR");
    // value below caption
    SliderIRDistance.setPrintValueProperties(11, FLAG_SLIDER_CAPTION_ALIGN_RIGHT | FLAG_SLIDER_CAPTION_BELOW,
            4 + TEXT_SIZE_10_HEIGHT, COLOR_BLACK, COLOR_WHITE);

    // Small US distance slider
    SliderUSDistance.init(BUTTON_WIDTH_6_POS_6 - (BUTTON_WIDTH_10 / 2) - 4, 10, (BUTTON_WIDTH_10 / 2) - 2, US_SLIDER_SIZE,
    DISTANCE_TIMEOUT_CM, 0, SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderUSDistance.setCaptionProperties(10, FLAG_SLIDER_CAPTION_ALIGN_LEFT | FLAG_SLIDER_CAPTION_BELOW, 2, COLOR_BLACK,
    COLOR_WHITE);
    SliderUSDistance.setCaption("US");
    // below caption
    SliderUSDistance.setPrintValueProperties(11, FLAG_SLIDER_CAPTION_ALIGN_LEFT | FLAG_SLIDER_CAPTION_BELOW,
            4 + TEXT_SIZE_10_HEIGHT, COLOR_BLACK, COLOR_WHITE);
#else
    // Big US distance slider
    SliderUSDistance.init(BUTTON_WIDTH_6_POS_6 - BUTTON_WIDTH_10 - 4, 10, BUTTON_WIDTH_10, US_SLIDER_SIZE, DISTANCE_TIMEOUT_CM, 0,
    SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderUSDistance.setBarThresholdColor(DISTANCE_TIMEOUT_COLOR);
    SliderUSDistance.setValueUnitString("cm");
#endif
    SliderUSDistance.setScaleFactor(2); // Slider is virtually 2 times larger, values were divided by 2
    SliderUSDistance.setBarThresholdColor(DISTANCE_TIMEOUT_COLOR);

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
        uint32_t tMillis = millis();

    if (tMillis >= sMillisOfNextVCCInfo) {
        sMillisOfNextVCCInfo = tMillis + PRINT_VOLTAGE_PERIOD_MILLIS;
        char tDataBuffer[18];
        char tVCCString[6];
        readVINVoltage();
        dtostrf(sVINVoltage, 4, 2, tVCCString);
        sprintf_P(tDataBuffer, PSTR("%s volt"), tVCCString);
        uint16_t tPosX;
        uint8_t tPosY;
        if (sCurrentPage == PAGE_HOME) {
            tPosX = BUTTON_WIDTH_8_POS_4;
            tPosY = BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER) - TEXT_SIZE_11_DECEND;
        }
        if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
            tPosX = BUTTON_WIDTH_3_POS_2 - BUTTON_DEFAULT_SPACING - (8 * TEXT_SIZE_11_WIDTH);
            tPosY = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;
        } else {
            tPosX = BUTTON_WIDTH_3_POS_3 - BUTTON_DEFAULT_SPACING - (8 * TEXT_SIZE_11_WIDTH);
            tPosY = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;
        }
        BlueDisplay1.drawText(tPosX, tPosY, tDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    }
}

void printMotorValues() {

    uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 25 + TEXT_SIZE_11_HEIGHT;
    sprintf_P(sStringBuffer, PSTR("min. %3d %3d"), leftEncoderMotor.MinSpeed, rightEncoderMotor.MinSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("max. %3d %3d"), leftEncoderMotor.MaxSpeed, rightEncoderMotor.MaxSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);

    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("comp. %2d  %2d"), leftEncoderMotor.SpeedCompensation, rightEncoderMotor.SpeedCompensation);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("act. %3d %3d"), leftEncoderMotor.ActualSpeed, rightEncoderMotor.ActualSpeed);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);

}

void printDistanceValues() {
    uint16_t tYPos;
    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
        tYPos = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;
    } else {
        tYPos = SPEED_SLIDER_SIZE / 2 + 25;
    }
    sprintf_P(sStringBuffer, PSTR("%4d %4d%3d"), leftEncoderMotor.DistanceCount, rightEncoderMotor.DistanceCount,
            sCentimeterPerScanTimesTwo);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}


void showUSDistance(unsigned int aCentimeter) {
// feedback as slider length
    if (aCentimeter != sSliderUSLastCentimeter) {
        sSliderUSLastCentimeter = aCentimeter;
        SliderUSDistance.setValueAndDrawBar(aCentimeter);
    }
}

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
void showIRDistance(unsigned int aCentimeter) {
// feedback as slider length
    if (aCentimeter != sSliderIRLastCentimeter) {
        sSliderIRLastCentimeter = aCentimeter;
        SliderIRDistance.setValueAndDrawBar(aCentimeter);
    }
}
#endif

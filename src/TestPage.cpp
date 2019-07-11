/*
 * RobotCarGui.cpp
 *
 *  Contains all common GUI elements for operating and controlling the RobotCar.
 *
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
BDButton TouchButtonReset;

BDButton TouchButtonGetAndStoreSpeed;

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

bool sShowDebug = false;

/*
 * UltraSonic control GUI
 */
BDSlider SliderUSPosition;
BDSlider SliderUSDistance;
unsigned int sSliderLastCentimeter;

unsigned int sLastCentimeterToObstacle = 0;
const int sGetDistancePeriod = 500;

void doDistance(BDButton * aTheTouchedButton, int16_t aValue) {
    if (!RobotCar.isDirectionForward) {
        aValue = -aValue;
    }
    RobotCar.initGoDistanceCentimeter(aValue);
}

void doShowDebug(BDButton * aTheTouchedButton, int16_t aValue) {
    sShowDebug = aValue;
}

void doRotation(BDButton * aTheTouchedButton, int16_t aValue) {
    RobotCar.initRotateCar(aValue, !RobotCar.isDirectionForward);
}

/*
 * Store ActualSpeed as MaxSpeed
 */
void doStoreSpeed(float aValue) {
    uint16_t tValue = aValue;
    if (tValue > 10 && tValue < 256) {
        // must use value for compensation not compensated value
        rightEncoderMotor.MaxSpeed = tValue;
        rightEncoderMotor.writeEeprom();

        // use the same value here !
        leftEncoderMotor.MaxSpeed = tValue;
        leftEncoderMotor.writeEeprom();
    }
    printMotorValues();
}

/*
 * Request speed value as number
 */
void doGetSpeedAsNumber(BDButton * aTheTouchedButton, int16_t aValue) {
    BlueDisplay1.getNumberWithShortPrompt(&doStoreSpeed, "Speed [11 - 255]", sLastSpeedSliderValue);
}

void doUSServoPosition(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    US_ServoWriteAndDelay(aValue);
}

void doReset(BDButton * aTheTouchedButton, int16_t aValue) {
    RobotCar.resetAndShutdownMotors();
    setDirectionButtonCaption();
    startStopRobotCar(false);
}

void initTestPage(void) {
    /*
     * scaled (0 to 180) US Sliders
     */
    SliderUSDistance.init(BUTTON_WIDTH_6_POS_6 - BUTTON_WIDTH_10 - 4, 10, BUTTON_WIDTH_10, US_SLIDER_SIZE, 200, 0,
    SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT,
    NULL);
    SliderUSDistance.setScaleFactor(2);
    SliderUSDistance.setValueUnitString("cm");

    SliderUSPosition.init(BUTTON_WIDTH_6_POS_6, 10, BUTTON_WIDTH_6, US_SLIDER_SIZE, 90, 90, COLOR_YELLOW, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE, &doUSServoPosition);
    SliderUSPosition.setBarThresholdColor(COLOR_BLUE);
    SliderUSPosition.setScaleFactor(180.0 / US_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderUSPosition.setValueUnitString("\xB0");

    /*
     * Control buttons
     */

    TouchButtonReset.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_BLUE, F("Reset"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doReset);

    TouchButtonGetAndStoreSpeed.init(0, BUTTON_HEIGHT_4_LINE_4 - BUTTON_HEIGHT_8_LINE_2 + 1, BUTTON_WIDTH_6, BUTTON_HEIGHT_8,
    COLOR_BLUE, F("Store"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doGetSpeedAsNumber);

    /*
     * Test buttons
     */
    TouchButton5cm.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, F("5cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 5, &doDistance);
    TouchButton10cm.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, F("10cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 10, &doDistance);

    TouchButton20cm.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, F("20cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 20, &doDistance);
    TouchButton40cm.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, F("40cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 40, &doDistance);

    TouchButton45DegreeLeft.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("45\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 45, &doRotation);
    TouchButton45DegreeRight.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("45\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, -45, &doRotation);

    TouchButton90DegreeLeft.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("90\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 90, &doRotation);
    TouchButton90DegreeRight.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("90\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, -90, &doRotation);
    TouchButton360Degree.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("360\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 360, &doRotation);

    TouchButtonDebug.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_RED, F("dbg"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sShowDebug, &doShowDebug);
}

void drawTestPage(void) {

    drawCommonGui();

    TouchButtonRobotCarStartStop.drawButton();
    TouchButtonReset.drawButton();
    TouchButtonBack.drawButton();

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
    TouchButtonGetAndStoreSpeed.drawButton();

    SliderUSPosition.setActualValueAndDrawBar(sLastServoAngleInDegrees);
    SliderUSPosition.drawSlider();
    SliderUSDistance.drawSlider();

    printMotorValues();
    if (sShowDebug) {
        printMotorDebugValues();
    }
    printDistanceValues();
}

void startTestPage(void) {
    drawTestPage();
}

void loopTestPage(void) {

    checkAndShowDistancePeriodically(sGetDistancePeriod);

    showSpeedSliderValue();

    if (EncoderMotor::ValuesHaveChanged) {
        EncoderMotor::ValuesHaveChanged = false;
        printMotorValues();
        if (sShowDebug) {
            printMotorDebugValues();
        }
    }

    displayVelocitySliderValues();

}

void stopTestPage(void) {

}

void showDistance(unsigned int aCentimeter) {
// feedback as slider length
    if (aCentimeter != sSliderLastCentimeter) {
        sSliderLastCentimeter = aCentimeter;
        SliderUSDistance.setActualValueAndDrawBar(aCentimeter);
    }
}

void checkAndShowDistancePeriodically(uint16_t aPeriodMillis) {
    if (rightEncoderMotor.State != MOTOR_STATE_RAMP_DOWN && rightEncoderMotor.State != MOTOR_STATE_RAMP_UP
            && leftEncoderMotor.State != MOTOR_STATE_RAMP_DOWN && leftEncoderMotor.State != MOTOR_STATE_RAMP_UP) {
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

void printMotorDebugValues() {
    /*
     * Debug info
     */
    uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 70;
    sprintf_P(sStringBuffer, PSTR("ramp1%3d %3d"), leftEncoderMotor.DistanceCountAfterRampUp,
            rightEncoderMotor.DistanceCountAfterRampUp);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("endSp%3d %3d"), leftEncoderMotor.SpeedAtTargetCountReached,
            rightEncoderMotor.SpeedAtTargetCountReached);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("debug%3d %3d"), leftEncoderMotor.Debug, rightEncoderMotor.Debug);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("dcnt %3d %3d"), leftEncoderMotor.DebugCount, rightEncoderMotor.DebugCount);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);

    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("tcnt %3d %3d"), leftEncoderMotor.LastTargetDistanceCount,
            rightEncoderMotor.LastTargetDistanceCount);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}


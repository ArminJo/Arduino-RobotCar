/*
 * RobotCarGui.cpp
 *
 *  Contains all common GUI elements for operating and controlling the RobotCarMotorControl.
 *
 *  Calibration: Sets lowest speed for which wheels are moving.
 *  Speed Slider left: Sets speed for manual control which serves also as maximum speed for autonomous drive if "Stored"
 *  Store: Stores calibration info and maximum speed.
 *  Cont ->Step / Step -> SStep, SStep->Cont: Switches mode from "continuous drive" to "drive until next turn" to "drive CENTIMETER_PER_RIDE_PRO"
 *  Start Simple: Start simple driving algorithm (using the 2 "simple" functions in RobotCarMotorControl.cpp)
 *  Start Pro: Start elaborated driving algorithm
 *
 *  insertToPath() and DrawPath() to show the path we were driving.
 *
 *  Requires BlueDisplay library.
 *
 *  Created on: 20.09.2016
 *  Copyright (C) 2016  Armin Joachimsmeyer
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

#include "RobotCar.h"
#include "RobotCarGui.h"

BDButton TouchButtonTestPage;
BDButton TouchButtonLaser;
#ifdef ENABLE_RTTTL
BDButton TouchButtonMelody;
#endif
#ifdef CAR_HAS_CAMERA
BDButton TouchButtonCameraOnOff;
#endif

#ifdef CAR_HAS_PAN_SERVO
BDSlider SliderLaserPositionHorizontal;
#endif
#ifdef CAR_HAS_TILT_SERVO
BDSlider SliderLaserPositionVertical;
#endif

#pragma GCC diagnostic ignored "-Wunused-parameter"

// Here we get values from 0 to 180 degrees from scaled slider
#ifdef CAR_HAS_PAN_SERVO
void doLaserServoPosition(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    LaserPanServo.write(aValue);
}
#endif

#ifdef CAR_HAS_TILT_SERVO
void doVerticalLaserServoPosition(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    TiltServo.write(aValue);
}
#endif

#ifdef CAR_HAS_LASER
void doLaserOnOff(BDButton * aTheTouchedButton, int16_t aValue) {
    digitalWrite(PIN_LASER_OUT, aValue);
}
#endif

#ifdef CAR_HAS_CAMERA
void doCameraSupplyOnOff(BDButton * aTheTouchedButton, int16_t aValue) {
    digitalWrite(PIN_CAMERA_SUPPLY_CONTROL, aValue);
}
#endif

#ifdef ENABLE_RTTTL
void doPlayMelody(BDButton * aTheTouchedButton, int16_t aValue) {
    sPlayMelody = aValue;
}
#endif

void initHomePage(void) {

    TouchButtonTestPage.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, F("Test"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, PAGE_TEST, &GUISwitchPages);

#ifdef CAR_HAS_CAMERA
    TouchButtonCameraOnOff.init(0, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER), BUTTON_WIDTH_3,
    TEXT_SIZE_22_HEIGHT, COLOR_BLACK, F("Camera"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            false, &doCameraSupplyOnOff);
#endif

#ifdef ENABLE_RTTTL
    TouchButtonMelody.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, BUTTON_HEIGHT_8, COLOR_BLACK, F("Melody"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false, &doPlayMelody);
#endif

#ifdef CAR_HAS_LASER
    TouchButtonLaser.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR_BLACK, F("Laser"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false, &doLaserOnOff);
#endif

#ifdef CAR_HAS_PAN_SERVO
    // left of SliderUSPosition
    SliderLaserPositionHorizontal.init(BUTTON_WIDTH_6_POS_6 - BUTTON_WIDTH_12 - BUTTON_DEFAULT_SPACING_HALF, SLIDER_TOP_MARGIN,
    BUTTON_WIDTH_12, LASER_SLIDER_SIZE, 90, 90, COLOR_YELLOW,
    SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doLaserServoPosition);
    SliderLaserPositionHorizontal.setBarThresholdColor(COLOR_BLUE);
    // scale slider values
    SliderLaserPositionHorizontal.setScaleFactor(180.0 / LASER_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderLaserPositionHorizontal.setValueUnitString("\xB0"); // \xB0 is degree character

    // initialize and set Laser pan servo
    LaserPanServo.attach(PIN_LASER_SERVO_PAN);
    LaserPanServo.write(90);
#endif

#ifdef CAR_HAS_TILT_SERVO
    SliderLaserPositionVertical.init(BUTTON_WIDTH_6_POS_6 - (2 * BUTTON_WIDTH_12) - BUTTON_DEFAULT_SPACING, SLIDER_TOP_MARGIN,
    BUTTON_WIDTH_12, LASER_SLIDER_SIZE, 90, 0, COLOR_YELLOW,
    SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doVerticalLaserServoPosition);
    SliderLaserPositionVertical.setBarThresholdColor(COLOR_BLUE);
    // scale slider values
    SliderLaserPositionVertical.setScaleFactor(180.0 / LASER_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderLaserPositionVertical.setValueUnitString("\xB0"); // \xB0 is degree character

    TiltServo.attach(PIN_LASER_SERVO_TILT);
    TiltServo.write(TILT_SERVO_MIN_VALUE);
#endif
}

/*
 * Manual control page
 */
void drawHomePage(void) {
    drawCommonGui();
    BlueDisplay1.drawText(HEADER_X + TEXT_SIZE_22_WIDTH, (2 * TEXT_SIZE_22_HEIGHT), F("Control"));

    char tCarTypeString[] = "4WD";
    if (RobotCarMotorControl.is2WDCar) {
        tCarTypeString[0] = '2';
    }

    BlueDisplay1.drawText(HEADER_X + (3 * TEXT_SIZE_22_WIDTH), (3 * TEXT_SIZE_22_HEIGHT), tCarTypeString);

    TouchButtonRobotCarStartStop.drawButton();
#ifdef ENABLE_RTTTL
    TouchButtonMelody.drawButton();
#endif
#ifdef CAR_HAS_CAMERA
    TouchButtonCameraOnOff.drawButton();
#endif
    TouchButtonTestPage.drawButton();
    TouchButtonNextPage.drawButton();

    TouchButtonDirection.drawButton();
    TouchButtonCalibrate.drawButton();

#ifdef CAR_HAS_LASER
    TouchButtonLaser.drawButton();

    SliderUSPosition.setValueAndDrawBar(sLastServoAngleInDegrees);
    SliderUSPosition.drawSlider();

#  ifdef CAR_HAS_PAN_SERVO
    SliderLaserPositionHorizontal.drawSlider();
#  endif
#  ifdef CAR_HAS_TILT_SERVO
    SliderLaserPositionVertical.drawSlider();
#  endif
#else
    SliderUSDistance.drawSlider();
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    SliderIRDistance.drawSlider();
#  endif
#endif
    SliderSpeed.drawSlider();
    SliderSpeedRight.drawSlider();
    SliderSpeedLeft.drawSlider();

    printMotorValues();

}

void startHomePage(void) {
    TouchButtonDirection.setPosition(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_4);
    TouchButtonCalibrate.setPosition(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_4);
    TouchButtonNextPage.setCaption(F("Automatic\nControl"));

    drawHomePage();
}

void loopHomePage(void) {
    displayVelocitySliderValues();

#ifndef CAR_HAS_LASER
    checkAndShowDistancePeriodically(sGetDistancePeriod);
#endif

    if (EncoderMotor::MotorValuesHaveChanged) {
        EncoderMotor::MotorValuesHaveChanged = false;
        printMotorValues();
    }
}

void stopHomePage(void) {
    TouchButtonDirection.setPosition(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_5);
    TouchButtonCalibrate.setPosition(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_2);
    startStopRobotCar(false);
}


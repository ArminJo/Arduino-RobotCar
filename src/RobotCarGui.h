/*
 * RobotCarGui.h
 *
 *  Created on: 20.09.2016
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
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

#ifndef SRC_ROBOTCARGUI_H_
#define SRC_ROBOTCARGUI_H_

#include "BlueDisplay.h"
#include "AutonomousDrive.h"

// can be deleted for BlueDisplay library version > 2.1.0
# if not defined(BUTTON_WIDTH_3_5_POS_2)
#define BUTTON_WIDTH_3_5_POS_2 (BUTTON_WIDTH_3_5 + BUTTON_DEFAULT_SPACING)
#define TEXT_SIZE_9   9
#endif

#define PATH_LENGTH_MAX 100

#define PRINT_VOLTAGE_PERIOD_MILLIS 3000
extern uint32_t sMillisOfNextVCCInfo;

// a string buffer for BD info output
extern char sStringBuffer[128];

/****************************************************************************
 * Change this if you have reprogrammed the hc05 module for other baud rate
 ***************************************************************************/
#ifndef BLUETOOTH_BAUD_RATE
//#define BLUETOOTH_BAUD_RATE BAUD_115200
#define BLUETOOTH_BAUD_RATE BAUD_9600
#endif

#define DISPLAY_WIDTH DISPLAY_HALF_VGA_WIDTH   // 320
#define DISPLAY_HEIGHT DISPLAY_HALF_VGA_HEIGHT // 240

#define HEADER_X BUTTON_WIDTH_3_5_POS_2

#define SLIDER_TOP_MARGIN 10
#define SPEED_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3  // 128
#define US_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3     // 128
#define LASER_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3  // 128
#define DISTANCE_SLIDER_SIZE (BUTTON_HEIGHT_4_LINE_3 - BUTTON_HEIGHT_8)  // 104

#define US_DISTANCE_MAP_ORIGIN_X 200
#define US_DISTANCE_MAP_ORIGIN_Y 150

#define PAGE_HOME 0 // Manual control page
#define PAGE_AUTOMATIC_CONTROL 1
#define PAGE_SHOW_PATH 2
#define PAGE_TEST 3
#define PAGE_LAST_NUMBER PAGE_TEST
//#define PAGE_FOLLOWER 4
//#define PAGE_LAST_NUMBER PAGE_FOLLOWER
extern uint8_t sCurrentPage;

void showUSDistance(unsigned int aCentimeter);
void showIRDistance(unsigned int aCentimeter);

// from PathInfoPage
void initPathInfoPage(void);
void drawPathInfoPage(void);
void startPathInfoPage(void);
void loopPathInfoPage(void);
void stopPathInfoPage(void);

// from AutonomousDrivePage
extern BDButton TouchButtonStep;
extern BDButton TouchButtonScanSpeed;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
extern BDButton TouchButtonScanMode;
#endif

void initAutonomousDrivePage(void);
void drawAutonomousDrivePage(void);
void startAutonomousDrivePage(void);
void loopAutonomousDrivePage(void);
void stopAutonomousDrivePage(void);

void handleAutomomousDriveRadioButtons();
void doStartStopFollowerMode(BDButton * aTheTouchedButton, int16_t aValue);
void doStartStopAutomomousDrive(BDButton * aTheTouchedButton, int16_t aValue);
void doStartStopTestUser(BDButton * aTheTouchedButton, int16_t aValue);

void doStartStopAutonomousForPathPage(BDButton * aTheTouchedButton, int16_t aValue);
void startStopAutomomousDrive(bool aDoStart, uint8_t aDriveMode = MODE_MANUAL_DRIVE);
void setStepMode(uint8_t aStepMode);

// from TestPage
void initTestPage(void);
void drawTestPage(void);
void startTestPage(void);
void loopTestPage(void);
void stopTestPage(void);
extern BDSlider SliderUSPosition;
extern BDSlider SliderUSDistance;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
extern BDSlider SliderIRDistance;
#endif

// from FollowerModePage
void initFollowerModePage(void);
void drawFollowerModePage(void);
void startFollowerModePage(void);
void loopFollowerModePage(void);
void stopFollowerModePage(void);

// from HomePage
extern BDButton TouchButtonMelody;

void initHomePage(void);
void drawHomePage(void);
void startHomePage(void);
void loopHomePage(void);
void stopHomePage(void);

/*
 * Page management
 */
extern uint8_t sCurrentPage;
extern BDButton TouchButtonNextPage;
extern BDButton TouchButtonReset;
extern BDButton TouchButtonBack;
extern BDButton TouchButtonBackSmall;
void GUISwitchPages(BDButton * aTheTouchedButton, int16_t aValue);
void startCurrentPage();

/*
 * Common GUI elements
 */
extern BDButton TouchButtonRobotCarStartStop;
void setStartStopButtonValue();
void startStopRobotCar(bool aNewStartedValue);
void doRobotCarStartStop(BDButton * aTheTochedButton, int16_t aValue);

extern BDButton TouchButtonDirection;
void doChangeDirection(BDButton * aTheTouchedButton, int16_t aValue);

extern BDButton TouchButtonCalibrate;
void doCalibrate(BDButton * aTheTouchedButton, int16_t aValue);

extern BDSlider SliderSpeed;
extern uint16_t sLastSpeedSliderValue;
void showSpeedSliderValue();

extern BDSlider SliderSpeedRight;
extern BDSlider SliderSpeedLeft;
void displayVelocitySliderValues();

void drawCommonGui(void);

extern char sStringBuffer[128];

void setupGUI(void);
void loopGUI(void);

void initRobotCarDisplay(void);
void checkAndShowDistancePeriodically(uint16_t aPeriodMillis);
void rotate(int16_t aRotationDegrees, bool inPlace = true);
void showDistance(int aCentimeter);

void printMotorValues();
void printMotorDebugValues();
void printDistanceValues();

void readAndPrintVin();
void readAndPrintVinPeriodically();
void delayAndLoopGUI(uint16_t aDelayMillis);

/*
 * Functions contained in RobotCarGuiOutput.cpp
 */
void DrawPath();
void resetPathData();
void insertToPath(int aLength, int aDegree, bool aAddEntry);

void clearPrintedForwardDistancesInfos();
void drawForwardDistancesInfos();
void drawCollisionDecision(int aDegreesToTurn, uint8_t aLengthOfVector, bool aDoClear);

extern uint8_t sRobotCarDirection;
extern bool sRuningAutonomousDrive;

extern const int sGetDistancePeriod;

#endif /* SRC_ROBOTCARGUI_H_ */

#pragma once

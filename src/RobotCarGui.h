/*
 * RobotCarGui.h
 *
 *  Created on: 20.09.2016
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 */

#ifndef SRC_ROBOTCARGUI_H_
#define SRC_ROBOTCARGUI_H_

#include "BlueDisplay.h"
#include "AutonomousDrive.h"

#define PATH_LENGTH_MAX 100

#define PRINT_VOLTAGE_PERIOD_MILLIS 3000

/****************************************************************************
 * Change this if you have reprogrammed the hc05 module for other baud rate
 ***************************************************************************/
#ifndef BLUETOOTH_BAUD_RATE
//#define BLUETOOTH_BAUD_RATE BAUD_115200
#define BLUETOOTH_BAUD_RATE BAUD_9600
#endif

#define DISPLAY_WIDTH DISPLAY_DEFAULT_WIDTH // 320
#define DISPLAY_HEIGHT DISPLAY_DEFAULT_HEIGHT // 240

#define SPEED_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3
#define US_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3 // 128
#define LASER_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3 // 128

#define US_DISTANCE_MAP_ORIGIN_X 200
#define US_DISTANCE_MAP_WIDTH_HALF 100
#define US_DISTANCE_MAP_ORIGIN_Y 150
#define US_DISTANCE_MAP_HEIGHT 100

#define PAGE_HOME 0 // Manual control page
#define PAGE_AUTOMATIC_CONTROL 1
#define PAGE_SHOW_PATH 2
#define PAGE_TEST 3
#define PAGE_LAST_NUMBER PAGE_TEST
extern uint8_t sCurrentPage;

#define MODE_CONTINUOUS 0
#define MODE_STEP_TO_NEXT_TURN 1 // stop before a turn
#define MODE_SINGLE_STEP 2 // stop after CENTIMETER_PER_RIDE_2
extern uint8_t sStepMode;
extern bool sDoStep;

// from PathInfoPage
void initPathInfoPage(void);
void drawPathInfoPage(void);
void startPathInfoPage(void);
void loopPathInfoPage(void);
void stopPathInfoPage(void);

// from AutonomousDrivePage
extern bool sUseBuiltInAutonomousDriveStrategy;
extern BDButton TouchButtonStep;

void initAutonomousDrivePage(void);
void drawAutonomousDrivePage(void);
void startAutonomousDrivePage(void);
void loopAutonomousDrivePage(void);
void stopAutonomousDrivePage(void);
void doStartStopAutomomousDrive(BDButton * aTheTouchedButton, int16_t aValue);
void doStartStopAutonomousForPathPage(BDButton * aTheTouchedButton, int16_t aValue);
void startStopAutomomousDrive(bool aDoStart, bool aDoInternalAutonomousDrive);
void setStepMode(uint8_t aStepMode);

// from TestPage
void initTestPage(void);
void drawTestPage(void);
void startTestPage(void);
void loopTestPage(void);
void stopTestPage(void);
extern BDSlider SliderUSPosition;
extern BDSlider SliderUSDistance;

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
void setDirectionButtonCaption();

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
//void resetGUIControls();

void initDisplay(void);
void checkAndShowDistancePeriodically(uint16_t aPeriodMillis);
void rotate(int16_t aRotationDegrees, bool inPlace = true);
void showDistance(int aCentimeter);

void printMotorValues();
void printMotorDebugValues();
void printDistanceValues();

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

extern bool sRobotCarStarted;
extern bool sRunAutonomousDrive;

extern const int sGetDistancePeriod;

#endif /* SRC_ROBOTCARGUI_H_ */

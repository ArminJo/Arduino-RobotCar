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

#define PATH_LENGTH_MAX 200

// Change this if you have reprogrammed the hc05 module for higher baud rate
//#define HC_05_BAUD_RATE BAUD_9600
#define HC_05_BAUD_RATE BAUD_115200
#define DISPLAY_WIDTH DISPLAY_DEFAULT_WIDTH // 320
#define DISPLAY_HEIGHT DISPLAY_DEFAULT_HEIGHT // 240

#define SPEED_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3
#define US_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3

#define SPEED_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3
#define US_SLIDER_SIZE BUTTON_HEIGHT_4_LINE_3 // 128

#define US_DISTANCE_MAP_ORIGIN_X 200
#define US_DISTANCE_MAP_WIDTH_HALF 100
#define US_DISTANCE_MAP_ORIGIN_Y 150
#define US_DISTANCE_MAP_HEIGHT 100

#define PAGE_MANUAL_CONTROL 0
#define PAGE_AUTOMATIC_CONTROL 1
#define PAGE_SHOW_PATH 2
extern uint8_t sActualPage;

#define MODE_CONTINUOUS 0
#define MODE_STEP 1 // stop before a turn
#define MODE_SINGLE_STEP 2 // stop after CENTIMETER_PER_RIDE_2
extern uint8_t sStepMode;
extern bool sDoStep;

extern char sDataBuffer[128];

void setupGUI(void);
void loopGUI(void);
void resetGUIControls();

void doStartStop(BDButton * aTheTochedButton, int16_t aValue);
void initDisplay(void);
void drawGui(void);
void checkAndShowDistancePeriodically(uint16_t aPeriodMillis);
void rotate(int16_t aRotationDegrees, bool inPlace = true);
void showDistance(int aCentimeter);

void DrawPath();
void resetPathData();
void insertToPath(int aDegree, int aLengthCentimeter);
void addToPath(int aLengthCentimeter, int aDegree);

void printMotorValues();
void printDistanceValues();
void printSingleDistanceVector(uint16_t aLength, int aDegree, Color_t aColor);
void printCollisionDecision(int aDegreeToTurn, uint8_t aLengthOfVector, bool aDoClear);

void drawVinPeriodically();

extern bool sRunManual;
extern bool sRunTestSimple;
extern bool sRunTestPro;

extern unsigned int sLastCentimeterToObstacle;
extern const int sGetDistancePeriod;

#endif /* SRC_ROBOTCARGUI_H_ */

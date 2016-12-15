/*
 * BlueDisplay.h
 *
 *   SUMMARY
 *  Blue Display is an Open Source Android remote Display for Arduino etc.
 *  It receives basic draw requests from Arduino etc. over Bluetooth and renders it.
 *  It also implements basic GUI elements as buttons and sliders.
 *  GUI callback, touch and sensor events are sent back to Arduino.
 *
 *  Copyright (C) 2014  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of BlueDisplay.
 *  BlueDisplay is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef BLUEDISPLAY_H_
#define BLUEDISPLAY_H_

#include "BlueDisplayProtocol.h"
// for convenience in order to include only BlueDisplay.h
#include "BlueSerial.h"
typedef uint16_t Color_t; // is needed in EventHandler.h
#include "EventHandler.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef AVR
#include <avr/pgmspace.h>
#endif

#ifdef __cplusplus
#include "BDButton.h" // for BDButtonHandle_t
#include "BDSlider.h" // for BDSliderHandle_t
#endif

/***************************
 * Origin 0.0 is upper left
 **************************/

#define DISPLAY_DEFAULT_HEIGHT 240 // value to use if not connected
#define DISPLAY_DEFAULT_WIDTH 320
#define STRING_BUFFER_STACK_SIZE 22 // Buffer size allocated on stack for ...PGM() functions.

/*
 * Basic colors
 */
// RGB to 16 bit 565 schema - 5 red | 6 green | 5 blue
#define COLOR_WHITE     ((Color_t)0xFFFF)
// 01 because 0 is used as flag (e.g. in touch button for default color)
#define COLOR_BLACK     ((Color_t)0X0001)
#define COLOR_RED       ((Color_t)0xF800)
#define COLOR_GREEN     ((Color_t)0X07E0)
#define COLOR_BLUE      ((Color_t)0x001F)
#define COLOR_DARK_BLUE ((Color_t)0x0014)
#define COLOR_YELLOW    ((Color_t)0XFFE0)
#define COLOR_ORANGE    ((Color_t)0XFE00)
#define COLOR_MAGENTA   ((Color_t)0xF81F)
#define COLOR_CYAN      ((Color_t)0x07FF)

// If used as background color for char or text, the background will not filled
#define COLOR_NO_BACKGROUND   ((Color_t)0XFFFE)

#define BLUEMASK 0x1F
#define GET_RED(rgb) ((rgb & 0xF800) >> 8)
#define GET_GREEN(rgb) ((rgb & 0x07E0) >> 3)
#define GET_BLUE(rgb) ((rgb & 0x001F) << 3)
#define RGB(r,g,b)   ((Color_t)(((r&0xF8)<<8)|((g&0xFC)<<3)|((b&0xF8)>>3))) //5 red | 6 green | 5 blue

#define SLIDER_DEFAULT_BORDER_COLOR     COLOR_BLUE
#define SLIDER_DEFAULT_BAR_COLOR        COLOR_GREEN
#define SLIDER_DEFAULT_BACKGROUND_COLOR COLOR_WHITE
#define SLIDER_DEFAULT_THRESHOLD_COLOR  COLOR_RED

/*
 * Android Text sizes which are closest to the 8*12 font used locally
 */
#define TEXT_SIZE_11 11
#define TEXT_SIZE_13 13
#define TEXT_SIZE_14 14
#define TEXT_SIZE_16 16
#define TEXT_SIZE_18 18
#define TEXT_SIZE_11 11
// for factor 2 of 8*12 font
#define TEXT_SIZE_22 22
// for factor 3 of 8*12 font
#define TEXT_SIZE_33 33
// for factor 4 of 8*12 font
#define TEXT_SIZE_44 44
// TextSize * 0.6
#ifdef LOCAL_DISPLAY_EXISTS
// 8/16 instead of 7/13 to be compatible with 8*12 font
#define TEXT_SIZE_11_WIDTH 8
#define TEXT_SIZE_22_WIDTH 16
#else
#define TEXT_SIZE_11_WIDTH 7
#define TEXT_SIZE_13_WIDTH 8
#define TEXT_SIZE_14_WIDTH 8
#define TEXT_SIZE_16_WIDTH 10
#define TEXT_SIZE_18_WIDTH 11
#define TEXT_SIZE_22_WIDTH 13
#endif

// TextSize * 0.93
// 12 instead of 11 to be compatible with 8*12 font and have a margin
#define TEXT_SIZE_11_HEIGHT 12
#define TEXT_SIZE_22_HEIGHT 24

// TextSize * 0.93
// 9 instead of 8 to have ASCEND + DECEND = HEIGHT
#define TEXT_SIZE_11_ASCEND 9
#define TEXT_SIZE_13_ASCEND 10
#define TEXT_SIZE_14_ASCEND 11
#define TEXT_SIZE_16_ASCEND 12
#define TEXT_SIZE_18_ASCEND 14
// 18 instead of 17 to have ASCEND + DECEND = HEIGHT
#define TEXT_SIZE_22_ASCEND 18

// TextSize * 0.24
#define TEXT_SIZE_11_DECEND 3
// 6 instead of 5 to have ASCEND + DECEND = HEIGHT
#define TEXT_SIZE_22_DECEND 6

uint16_t getTextHeight(uint16_t aTextSize);
uint16_t getTextWidth(uint16_t aTextSize);
uint16_t getTextAscend(uint16_t aTextSize);
uint16_t getTextAscendMinusDescend(uint16_t aTextSize);
uint16_t getTextMiddle(uint16_t aTextSize);
uint16_t getLocalTextSize(uint16_t aTextSize);

/**********************
 * BUTTON LAYOUTS
 *********************/
#define BUTTON_AUTO_RED_GREEN_FALSE_COLOR COLOR_RED
#define BUTTON_AUTO_RED_GREEN_TRUE_COLOR COLOR_GREEN
#define BUTTON_DEFAULT_SPACING 16
#define BUTTON_DEFAULT_SPACING_THREE_QUARTER 12
#define BUTTON_DEFAULT_SPACING_HALF 8
#define BUTTON_DEFAULT_SPACING_QUARTER 4
/*
 * Layout for 320 x 240 screen size
 */
#define LAYOUT_320_WIDTH 320
#define LAYOUT_240_HEIGHT 240
#define LAYOUT_256_HEIGHT 256

/**********************
 * WIDTHS
 *********************/
// For documentation
#ifdef __cplusplus
#if ( false ) // It can only be used with compiler flag -std=gnu++11
constexpr int ButtonWidth ( int aNumberOfButtonsPerLine, int aDisplayWidth ) {return ((aDisplayWidth - ((aNumberOfButtonsPerLine-1)*aDisplayWidth/20))/aNumberOfButtonsPerLine);}
#endif
#endif

#define BUTTON_HORIZONTAL_SPACING_DYN (sActualDisplayWidth/64)

// for 2 buttons horizontal - 19 characters
#define BUTTON_WIDTH_2 152
#define BUTTON_WIDTH_2_POS_2 (BUTTON_WIDTH_2 + BUTTON_DEFAULT_SPACING)
//
// for 3 buttons horizontal - 12 characters
#define BUTTON_WIDTH_3 96
#define BUTTON_WIDTH_3_POS_2 (BUTTON_WIDTH_3 + BUTTON_DEFAULT_SPACING)
#define BUTTON_WIDTH_3_POS_3 (LAYOUT_320_WIDTH - BUTTON_WIDTH_3)
//
// for 3 buttons horizontal - dynamic
#define BUTTON_WIDTH_3_DYN (sActualDisplayWidth/3 - BUTTON_HORIZONTAL_SPACING_DYN)
#define BUTTON_WIDTH_3_DYN_POS_2 (sActualDisplayWidth/3)
#define BUTTON_WIDTH_3_DYN_POS_3 (sActualDisplayWidth - BUTTON_WIDTH_3_DYN)

#define BUTTON_WIDTH_3_5 82
//
// for 4 buttons horizontal - 8 characters
#define BUTTON_WIDTH_4 68
#define BUTTON_WIDTH_4_POS_2 (BUTTON_WIDTH_4 + BUTTON_DEFAULT_SPACING)
#define BUTTON_WIDTH_4_POS_3 (2*(BUTTON_WIDTH_4 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_4_POS_4 (LAYOUT_320_WIDTH - BUTTON_WIDTH_4)//
//
// for 4 buttons horizontal - dynamic
#define BUTTON_WIDTH_4_DYN (sActualDisplayWidth/4 - BUTTON_HORIZONTAL_SPACING_DYN)
#define BUTTON_WIDTH_4_DYN_POS_2 (sActualDisplayWidth/4)
#define BUTTON_WIDTH_4_DYN_POS_3 (sActualDisplayWidth/2)
#define BUTTON_WIDTH_4_DYN_POS_4 (sActualDisplayWidth - BUTTON_WIDTH_4_DYN)
//
// for 5 buttons horizontal 51,2  - 6 characters
#define BUTTON_WIDTH_5 51
#define BUTTON_WIDTH_5_POS_2 (BUTTON_WIDTH_5 + BUTTON_DEFAULT_SPACING)
#define BUTTON_WIDTH_5_POS_3 (2*(BUTTON_WIDTH_5 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_5_POS_4 (3*(BUTTON_WIDTH_5 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_5_POS_5 (LAYOUT_320_WIDTH - BUTTON_WIDTH_5)
//
//  for 2 buttons horizontal plus one small with BUTTON_WIDTH_5 (118,5)- 15 characters
#define BUTTON_WIDTH_2_5 120
#define BUTTON_WIDTH_2_5_POS_2   (BUTTON_WIDTH_2_5 + BUTTON_DEFAULT_SPACING -1)
#define BUTTON_WIDTH_2_5_POS_2_5 (LAYOUT_320_WIDTH - BUTTON_WIDTH_5)
//
// for 6 buttons horizontal
#define BUTTON_WIDTH_6 40
#define BUTTON_WIDTH_6_POS_2 (BUTTON_WIDTH_6 + BUTTON_DEFAULT_SPACING)
#define BUTTON_WIDTH_6_POS_3 (2*(BUTTON_WIDTH_6 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_6_POS_4 (3*(BUTTON_WIDTH_6 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_6_POS_5 (4*(BUTTON_WIDTH_6 + BUTTON_DEFAULT_SPACING))
#define BUTTON_WIDTH_6_POS_6 (LAYOUT_320_WIDTH - BUTTON_WIDTH_6)
//
// for 8 buttons horizontal
#define BUTTON_WIDTH_8 33
#define BUTTON_WIDTH_8_POS_2 (BUTTON_WIDTH_8 + BUTTON_DEFAULT_SPACING_HALF)
#define BUTTON_WIDTH_8_POS_3 (2*(BUTTON_WIDTH_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_WIDTH_8_POS_4 (3*(BUTTON_WIDTH_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_WIDTH_8_POS_5 (4*(BUTTON_WIDTH_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_WIDTH_8_POS_6 (5*(BUTTON_WIDTH_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_WIDTH_8_POS_7 (6*(BUTTON_WIDTH_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_WIDTH_8_POS_8 (LAYOUT_320_WIDTH - BUTTON_WIDTH_0)
//
// for 10 buttons horizontal
#define BUTTON_WIDTH_10 28
#define BUTTON_WIDTH_10_POS_2 (BUTTON_WIDTH_10 + BUTTON_DEFAULT_SPACING_QUARTER)
#define BUTTON_WIDTH_10_POS_3 (2*(BUTTON_WIDTH_10 + BUTTON_DEFAULT_SPACING_QUARTER))
#define BUTTON_WIDTH_10_POS_4 (3*(BUTTON_WIDTH_10 + BUTTON_DEFAULT_SPACING_QUARTER))
#define BUTTON_WIDTH_10_POS_5 (4*(BUTTON_WIDTH_10 + BUTTON_DEFAULT_SPACING_QUARTER))
#define BUTTON_WIDTH_10_POS_6 (5*(BUTTON_WIDTH_10 + BUTTON_DEFAULT_SPACING_QUARTER))
#define BUTTON_WIDTH_10_POS_7 (6*(BUTTON_WIDTH_10 + BUTTON_DEFAULT_SPACING_QUARTER))
#define BUTTON_WIDTH_10_POS_8 (7*(BUTTON_WIDTH_10 + BUTTON_DEFAULT_SPACING_QUARTER))
#define BUTTON_WIDTH_10_POS_9 (8*(BUTTON_WIDTH_10 + BUTTON_DEFAULT_SPACING_QUARTER))
#define BUTTON_WIDTH_10_POS_10 (LAYOUT_320_WIDTH - BUTTON_WIDTH_0)

#define BUTTON_WIDTH_16 16
#define BUTTON_WIDTH_16_POS_2 (BUTTON_WIDTH_16 + BUTTON_DEFAULT_SPACING_QUARTER)

/**********************
 * HEIGHTS
 *********************/
#define BUTTON_VERTICAL_SPACING_DYN (sActualDisplayHeight/32)

// for 4 buttons vertical
#define BUTTON_HEIGHT_4 48
#define BUTTON_HEIGHT_4_LINE_2 (BUTTON_HEIGHT_4 + BUTTON_DEFAULT_SPACING)
#define BUTTON_HEIGHT_4_LINE_3 (2*(BUTTON_HEIGHT_4 + BUTTON_DEFAULT_SPACING))
#define BUTTON_HEIGHT_4_LINE_4 (LAYOUT_240_HEIGHT - BUTTON_HEIGHT_4)
//
// for 4 buttons vertical and DISPLAY_HEIGHT 256
#define BUTTON_HEIGHT_4_256 52
#define BUTTON_HEIGHT_4_256_LINE_2 (BUTTON_HEIGHT_4_256 + BUTTON_DEFAULT_SPACING)
#define BUTTON_HEIGHT_4_256_LINE_3 (2*(BUTTON_HEIGHT_4_256 + BUTTON_DEFAULT_SPACING))
#define BUTTON_HEIGHT_4_256_LINE_4 (LAYOUT_256_HEIGHT - BUTTON_HEIGHT_4_256)
//
// for 4 buttons vertical and variable display height
#define BUTTON_HEIGHT_4_DYN (sActualDisplayHeight/4 - BUTTON_VERTICAL_SPACING_DYN)
#define BUTTON_HEIGHT_4_DYN_LINE_2 (sActualDisplayHeight/4)
#define BUTTON_HEIGHT_4_DYN_LINE_3 (sActualDisplayHeight/2)
#define BUTTON_HEIGHT_4_DYN_LINE_4 (sActualDisplayHeight - BUTTON_HEIGHT_4_DYN)
//
// for 5 buttons vertical
#define BUTTON_HEIGHT_5 38
#define BUTTON_HEIGHT_5_LINE_2 (BUTTON_HEIGHT_5 + BUTTON_DEFAULT_SPACING_THREE_QUARTER)
#define BUTTON_HEIGHT_5_LINE_3 (2*(BUTTON_HEIGHT_5 + BUTTON_DEFAULT_SPACING_THREE_QUARTER))
#define BUTTON_HEIGHT_5_LINE_4 (3*(BUTTON_HEIGHT_5 + BUTTON_DEFAULT_SPACING_THREE_QUARTER))
#define BUTTON_HEIGHT_5_LINE_5 (LAYOUT_240_HEIGHT - BUTTON_HEIGHT_5)
//
// for 5 buttons vertical and DISPLAY_HEIGHT 256
#define BUTTON_HEIGHT_5_256 39
#define BUTTON_HEIGHT_5_256_LINE_2 (BUTTON_HEIGHT_5 + BUTTON_DEFAULT_SPACING)
#define BUTTON_HEIGHT_5_256_LINE_3 (2*(BUTTON_HEIGHT_5 + BUTTON_DEFAULT_SPACING))
#define BUTTON_HEIGHT_5_256_LINE_4 (3*(BUTTON_HEIGHT_5 + BUTTON_DEFAULT_SPACING))
#define BUTTON_HEIGHT_5_256_LINE_5 (LAYOUT_256_HEIGHT - BUTTON_HEIGHT_5)
//
// for 5 buttons vertical and variable display height
#define BUTTON_HEIGHT_5_DYN (sActualDisplayHeight/5 - BUTTON_VERTICAL_SPACING_DYN)
#define BUTTON_HEIGHT_5_DYN_LINE_2 (sActualDisplayHeight/5)
#define BUTTON_HEIGHT_5_DYN_LINE_3 ((sActualDisplayHeight/5)*2)
#define BUTTON_HEIGHT_5_DYN_LINE_4 ((sActualDisplayHeight/5)*3)
#define BUTTON_HEIGHT_5_DYN_LINE_5 (sActualDisplayHeight - BUTTON_HEIGHT_5_DYN)
//
// for 6 buttons vertical
#define BUTTON_HEIGHT_6 30
#define BUTTON_HEIGHT_6_LINE_2 (BUTTON_HEIGHT_6 + BUTTON_DEFAULT_SPACING_THREE_QUARTER)
#define BUTTON_HEIGHT_6_LINE_3 (2*(BUTTON_HEIGHT_6 + BUTTON_DEFAULT_SPACING_THREE_QUARTER))
#define BUTTON_HEIGHT_6_LINE_4 (3*(BUTTON_HEIGHT_6 + BUTTON_DEFAULT_SPACING_THREE_QUARTER))
#define BUTTON_HEIGHT_6_LINE_5 (4*(BUTTON_HEIGHT_6 + BUTTON_DEFAULT_SPACING_THREE_QUARTER))
#define BUTTON_HEIGHT_6_LINE_6 (LAYOUT_HEIGHT - BUTTON_HEIGHT_6)

// for 8 buttons vertical
#define BUTTON_HEIGHT_8 29
#define BUTTON_HEIGHT_8_LINE_2 (BUTTON_HEIGHT_8 + BUTTON_DEFAULT_SPACING_HALF)
#define BUTTON_HEIGHT_8_LINE_3 (2*(BUTTON_HEIGHT_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_HEIGHT_8_LINE_4 (3*(BUTTON_HEIGHT_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_HEIGHT_8_LINE_5 (4*(BUTTON_HEIGHT_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_HEIGHT_8_LINE_6 (5*(BUTTON_HEIGHT_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_HEIGHT_8_LINE_7 (6*(BUTTON_HEIGHT_8 + BUTTON_DEFAULT_SPACING_HALF))
#define BUTTON_HEIGHT_8_LINE_8 (LAYOUT_HEIGHT - BUTTON_HEIGHT_8)

/**********************
 * Constants used in protocol
 *********************/
//#define COLOR_NO_BACKGROUND   ((Color_t)0XFFFE)
static const float NUMBER_INITIAL_VALUE_DO_NOT_SHOW = 1e-40f;

/**********************
 * Basic
 *********************/
// Sub functions for SET_FLAGS_AND_SIZE
// Reset buttons, sliders, sensors, orientation locking, flags (see next lines) and character mappings
static const int BD_FLAG_FIRST_RESET_ALL = 0x01;
// disables also touch moves
static const int BD_FLAG_TOUCH_BASIC_DISABLE = 0x02;
static const int BD_FLAG_ONLY_TOUCH_MOVE_DISABLE = 0x04;
static const int BD_FLAG_LONG_TOUCH_ENABLE = 0x08;
static const int BD_FLAG_USE_MAX_SIZE = 0x10;

/**********************
 * Button
 *********************/
// Flags for BUTTON_GLOBAL_SETTINGS
// old
static const int USE_UP_EVENTS_FOR_BUTTONS = 0x01;
static const int BUTTONS_SET_BEEP_TONE = 0x02;
// new
static const int FLAG_BUTTON_GLOBAL_USE_DOWN_EVENTS_FOR_BUTTONS = 0x00;
static const int FLAG_BUTTON_GLOBAL_USE_UP_EVENTS_FOR_BUTTONS = 0x01;
static const int FLAG_BUTTON_GLOBAL_SET_BEEP_TONE = 0x02;

// Flags for init
//old
static const int BUTTON_FLAG_NO_BEEP_ON_TOUCH = 0x00;
static const int BUTTON_FLAG_DO_BEEP_ON_TOUCH = 0x01;
static const int BUTTON_FLAG_TYPE_AUTO_RED_GREEN = 0x02; // see void doToggleRedGreenButton() / BDButton.cpp
static const int BUTTON_FLAG_TYPE_AUTOREPEAT = 0x04;
//new
static const int FLAG_BUTTON_NO_BEEP_ON_TOUCH = 0x00;
static const int FLAG_BUTTON_DO_BEEP_ON_TOUCH = 0x01;
static const int FLAG_BUTTON_TYPE_AUTO_RED_GREEN = 0x02;
static const int FLAG_BUTTON_TYPE_AUTOREPEAT = 0x04;
#ifdef AVR
static const int FLAG_BUTTON_CAPTION_IS_IN_PGMSPACE = 0x40;
#endif

#ifdef LOCAL_DISPLAY_EXISTS
#define LOCAL_BUTTON_FLAG_MASK 0x18
#define FLAG_USE_BDBUTTON_FOR_CALLBACK 0x08
#define FLAG_IS_ACTIVE 0x10 // Button is checked by checkAllButtons() etc.
#endif

#ifdef USE_BUTTON_POOL
#define INTERNAL_FLAG_MASK 0x80
#define FLAG_IS_ALLOCATED 0x80 // for use with get and releaseButton
#endif

/**********************
 * Slider
 *********************/
// Flags for slider options
static const int FLAG_SLIDER_VERTICAL = 0x00;
static const int FLAG_SLIDER_VERTICAL_SHOW_NOTHING = 0x00;
static const int FLAG_SLIDER_SHOW_BORDER = 0x01;
// if set, ASCII value is printed along with change of bar value
static const int FLAG_SLIDER_SHOW_VALUE = 0x02;
static const int FLAG_SLIDER_IS_HORIZONTAL = 0x04;
static const int FLAG_SLIDER_IS_INVERSE = 0x08;
// if set,  bar (+ ASCII) value will be set by callback handler, not by touch
static const int FLAG_SLIDER_VALUE_BY_CALLBACK = 0x10;
static const int FLAG_SLIDER_IS_ONLY_OUTPUT = 0x20;

// Flags for slider caption position
static const int FLAG_SLIDER_CAPTION_ALIGN_LEFT_BELOW = 0x00;
static const int FLAG_SLIDER_CAPTION_ALIGN_LEFT = 0x00;
static const int FLAG_SLIDER_CAPTION_ALIGN_RIGHT = 0x01;
static const int FLAG_SLIDER_CAPTION_ALIGN_MIDDLE = 0x02;
static const int FLAG_SLIDER_CAPTION_BELOW = 0x00;
static const int FLAG_SLIDER_CAPTION_ABOVE = 0x04;

/**********************
 * Tone
 *********************/
// Android system tones
// codes start with 0 - 15 for DTMF tones and ends with code TONE_CDMA_SIGNAL_OFF=98 for silent tone (which does not work on lollipop)
#define TONE_CDMA_KEYPAD_VOLUME_KEY_LITE 89
#define TONE_PROP_BEEP 27
#define TONE_PROP_BEEP2 28
#define TONE_SILENCE 50 // since 98 does not work on lollipop
#define TONE_CDMA_ONE_MIN_BEEP 88
#define TONE_DEFAULT TONE_CDMA_KEYPAD_VOLUME_KEY_LITE
#define TONE_LAST_VALID_TONE_INDEX 98

/**********************
 * Sensors
 *********************/
// see android.hardware.Sensor
static const int FLAG_SENSOR_TYPE_ACCELEROMETER = 1;
static const int FLAG_SENSOR_TYPE_GYROSCOPE = 4;

// rate of sensor callbacks - see android.hardware.SensorManager
static const int FLAG_SENSOR_DELAY_NORMAL = 3; // 200 ms
static const int FLAG_SENSOR_DELAY_UI = 2; // 60 ms
static const int FLAG_SENSOR_DELAY_GAME = 1; // 20 ms
static const int FLAG_SENSOR_DELAY_FASTEST = 0;
static const int FLAG_SENSOR_NO_FILTER = 0;
static const int FLAG_SENSOR_SIMPLE_FILTER = 1;

// Flags for SET_SCREEN_ORIENTATION_LOCK
static const int FLAG_SCREEN_ORIENTATION_LOCK_LANDSCAPE = 0x00;
static const int FLAG_SCREEN_ORIENTATION_LOCK_PORTRAIT = 0x01;
static const int FLAG_SCREEN_ORIENTATION_LOCK_ACTUAL = 0x02;
static const int FLAG_SCREEN_ORIENTATION_LOCK_UNLOCK = 0x03;

// no valid button number
#define NO_BUTTON 0xFF
#define NO_SLIDER 0xFF

struct ThickLine {
    int16_t StartX;
    int16_t StartY;
    int16_t EndX;
    int16_t EndY;
    int16_t Thickness;
    Color_t Color;
    Color_t BackgroundColor;
};

#ifdef __cplusplus
#define MAX_NUMBER_OF_ARGS_FOR_BD_FUNCTIONS 12 // for sending

class BlueDisplay {
public:
    BlueDisplay();
    void resetLocal(void);
    void initCommunication(void (*aConnectCallback)(void), void (*aReorientationCallback)(void), void (*aRedrawCallback)(void));
    // with combined callbacks
    void initCommunication(void (*aConnectAndReorientationCallback)(void), void (*aRedrawCallback)(void));
    void sendSync(void);
    void setFlagsAndSize(uint16_t aFlags, uint16_t aWidth, uint16_t aHeight);
    void setCodePage(uint16_t aCodePageNumber);
    void setCharacterMapping(uint8_t aChar, uint16_t aUnicodeChar); // aChar must be bigger than 0x80

    void playTone(void);
    void playTone(uint8_t aToneIndex);
    void playTone(uint8_t aToneIndex, int16_t aToneDuration);
    void playTone(uint8_t aToneIndex, int16_t aToneDuration, uint8_t aToneVolume);
    void playFeedbackTone(bool isError);
    void setLongTouchDownTimeout(uint16_t aLongTouchDownTimeoutMillis);

    void clearDisplay(Color_t aColor);
    void drawDisplayDirect(void);
    void setScreenOrientationLock(uint8_t aLockMode);

    void drawPixel(uint16_t aXPos, uint16_t aYPos, Color_t aColor);
    void drawCircle(uint16_t aXCenter, uint16_t aYCenter, uint16_t aRadius, Color_t aColor, uint16_t aStrokeWidth);
    void fillCircle(uint16_t aXCenter, uint16_t aYCenter, uint16_t aRadius, Color_t aColor);
    void drawRect(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, Color_t aColor, uint16_t aStrokeWidth);
    void drawRectRel(uint16_t aXStart, uint16_t aYStart, uint16_t aWidth, uint16_t aHeight, Color_t aColor, uint16_t aStrokeWidth);
    void fillRect(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, Color_t aColor);
    void fillRectRel(uint16_t aXStart, uint16_t aYStart, uint16_t aWidth, uint16_t aHeight, Color_t aColor);
    uint16_t drawChar(uint16_t aPosX, uint16_t aPosY, char aChar, uint16_t aCharSize, Color_t aFGColor, Color_t aBGColor);
    uint16_t drawText(uint16_t aXStart, uint16_t aYStart, const char *aStringPtr, uint16_t aFontSize, Color_t aFGColor,
            Color_t aBGColor);

    uint16_t drawByte(uint16_t aPosX, uint16_t aPosY, int8_t aByte, uint16_t aTextSize = TEXT_SIZE_11, Color_t aFGColor =
    COLOR_BLACK, Color_t aBGColor = COLOR_WHITE);
    uint16_t drawUnsignedByte(uint16_t aPosX, uint16_t aPosY, uint8_t aUnsignedByte, uint16_t aTextSize = TEXT_SIZE_11,
            Color_t aFGColor = COLOR_BLACK, Color_t aBGColor = COLOR_WHITE);
    uint16_t drawShort(uint16_t aPosX, uint16_t aPosY, int16_t aShort, uint16_t aTextSize = TEXT_SIZE_11, Color_t aFGColor =
    COLOR_BLACK, Color_t aBGColor = COLOR_WHITE);
    uint16_t drawLong(uint16_t aPosX, uint16_t aPosY, int32_t aLong, uint16_t aTextSize = TEXT_SIZE_11, Color_t aFGColor =
    COLOR_BLACK, Color_t aBGColor = COLOR_WHITE);

    void setPrintfSizeAndColorAndFlag(uint16_t aPrintSize, Color_t aPrintColor, Color_t aPrintBackgroundColor,
    bool aClearOnNewScreen);
    void setPrintfPosition(uint16_t aPosX, uint16_t aPosY);
    void setPrintfPositionColumnLine(uint16_t aColumnNumber, uint16_t aLineNumber);
    void writeString(const char *aStringPtr, uint8_t aStringLength);

    void debugMessage(const char *aStringPtr);
    void debug(uint8_t aByte);
    void debug(int8_t aByte);
    void debug(uint16_t aShort);
    void debug(int aShort);
    void debug(uint32_t aShort);
    void debug(double aDouble);

    void drawLine(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, Color_t aColor);
    void drawLineRel(uint16_t aXStart, uint16_t aYStart, uint16_t aXDelta, uint16_t aYDelta, Color_t aColor);
    void drawLineFastOneX(uint16_t x0, uint16_t y0, uint16_t y1, Color_t aColor);
    void drawVectorDegree(uint16_t aXStart, uint16_t aYStart, uint16_t aLength, int aDegree, Color_t aColor,
            int16_t aThickness = 1);
    void drawVectorRadian(uint16_t aXStart, uint16_t aYStart, uint16_t aLength, int aRadian, Color_t aColor,
            int16_t aThickness = 1);
    void drawLineWithThickness(uint16_t aXStart, uint16_t aYStart, uint16_t aXEnd, uint16_t aYEnd, int16_t aThickness,
            Color_t aColor);
    void drawLineRelWithThickness(uint16_t aXStart, uint16_t aYStart, uint16_t aXDelta, uint16_t aYDelta, int16_t aThickness,
            Color_t aColor);

    void drawChartByteBuffer(uint16_t aXOffset, uint16_t aYOffset, Color_t aColor, Color_t aClearBeforeColor, uint8_t *aByteBuffer,
            size_t aByteBufferLength);
    void drawChartByteBuffer(uint16_t aXOffset, uint16_t aYOffset, Color_t aColor, Color_t aClearBeforeColor, uint8_t aChartIndex,
    bool aDoDrawDirect, uint8_t *aByteBuffer, size_t aByteBufferLength);

    struct XYSize * getMaxDisplaySize(void);
    uint16_t getMaxDisplayWidth(void);
    uint16_t getMaxDisplayHeight(void);
    struct XYSize * getActualDisplaySize(void);
    uint16_t getActualDisplayWidth(void);
    uint16_t getActualDisplayHeight(void);
    // returns requested size
    struct XYSize * getReferenceDisplaySize(void);
    uint16_t getDisplayWidth(void);
    uint16_t getDisplayHeight(void);
    // implemented by event handler
    bool isDisplayOrientationLandscape(void);

    void refreshVector(struct ThickLine * aLine, int16_t aNewRelEndX, int16_t aNewRelEndY);

    void getNumber(void (*aNumberHandler)(float));
    void getNumberWithShortPrompt(void (*aNumberHandler)(float), const char *aShortPromptString);
    void getNumberWithShortPrompt(void (*aNumberHandler)(float), const char *aShortPromptString, float aInitialValue);
    // Not yet implemented
    //    void getText(void (*aTextHandler)(const char *));
    //    void getTextWithShortPrompt(void (*aTextHandler)(const char *), const char *aShortPromptString);
    void getInfo(uint16_t aInfoSubcommand, void (*aInfoHandler)(uint8_t *));
    // results in a reorientation callback
    void requestMaxCanvasSize(void);

    void setSensor(uint8_t aSensorType, bool aDoActivate, uint8_t aSensorRate, uint8_t aFilterFlag);

#ifdef LOCAL_DISPLAY_EXISTS
    void drawMLText(uint16_t aPosX, uint16_t aPosY, const char *aStringPtr, uint16_t aTextSize, Color_t aFGColor, Color_t aBGColor);
#endif

#ifdef AVR
    uint16_t drawTextPGM(uint16_t aXStart, uint16_t aYStart, const char * aPGMString, uint8_t aFontSize, Color_t aFGColor,
            Color_t aBGColor);
    void getNumberWithShortPromptPGM(void (*aNumberHandler)(float), const char *aPGMShortPromptString);
    void getNumberWithShortPromptPGM(void (*aNumberHandler)(float), const char *aPGMShortPromptString, float aInitialValue);
    // Not yet implemented    void getTextWithShortPromptPGM(void (*aTextHandler)(const char *), const char *aPGMShortPromptString);

    void printVCCAndTemperaturePeriodically(uint16_t aXPos, uint16_t aYPos, uint8_t aTextSize, uint16_t aPeriodMillis);
#endif
    /*
     * Button stuff
     */
    BDButtonHandle_t createButton(uint16_t aPositionX, uint16_t aPositionY, uint16_t aWidthX, uint16_t aHeightY,
            Color_t aButtonColor, const char * aCaption, uint8_t aCaptionSize, uint8_t aFlags, int16_t aValue,
            void (*aOnTouchHandler)(BDButton *, int16_t));
    void drawButton(BDButtonHandle_t aButtonNumber);
    void removeButton(BDButtonHandle_t aButtonNumber, Color_t aBackgroundColor);
    void drawButtonCaption(BDButtonHandle_t aButtonNumber);
    void setButtonCaption(BDButtonHandle_t aButtonNumber, const char * aCaption, bool doDrawButton);
    void setButtonValue(BDButtonHandle_t aButtonNumber, int16_t aValue);
    void setButtonValueAndDraw(BDButtonHandle_t aButtonNumber, int16_t aValue);
    void setButtonColor(BDButtonHandle_t aButtonNumber, Color_t aButtonColor);
    void setButtonColorAndDraw(BDButtonHandle_t aButtonNumber, Color_t aButtonColor);
    void setButtonPosition(BDButtonHandle_t aButtonNumber, int16_t aPositionX, int16_t aPositionY);
    void setButtonAutorepeatTiming(BDButtonHandle_t aButtonNumber, uint16_t aMillisFirstDelay, uint16_t aMillisFirstRate,
            uint16_t aFirstCount, uint16_t aMillisSecondRate);

    void activateButton(BDButtonHandle_t aButtonNumber);
    void deactivateButton(BDButtonHandle_t aButtonNumber);
    void activateAllButtons(void);
    void deactivateAllButtons(void);
    void setButtonsGlobalFlags(uint16_t aFlags);
    void setButtonsTouchTone(uint8_t aToneIndex, uint8_t aToneVolume);

#ifdef AVR
    BDButtonHandle_t createButtonPGM(uint16_t aPositionX, uint16_t aPositionY, uint16_t aWidthX, uint16_t aHeightY,
            Color_t aButtonColor, const char * aPGMCaption, uint8_t aCaptionSize, uint8_t aFlags, int16_t aValue,
            void (*aOnTouchHandler)(BDButton *, int16_t));
    void setButtonCaptionPGM(BDButtonHandle_t aButtonNumber, const char * aPGMCaption, bool doDrawButton);
#endif

    /*
     * Slider stuff
     */
    BDSliderHandle_t createSlider(uint16_t aPositionX, uint16_t aPositionY, uint8_t aBarWidth, int16_t aBarLength,
            int16_t aThresholdValue, int16_t aInitalValue, Color_t aSliderColor, Color_t aBarColor, uint8_t aFlags,
            void (*aOnChangeHandler)(BDSliderHandle_t *, int16_t));
    void drawSlider(BDSliderHandle_t aSliderNumber);
    void drawSliderBorder(BDSliderHandle_t aSliderNumber);
    void setSliderActualValueAndDrawBar(BDSliderHandle_t aSliderNumber, int16_t aActualValue);
    void setSliderColorBarThreshold(BDSliderHandle_t aSliderNumber, uint16_t aBarThresholdColor);
    void setSliderColorBarBackground(BDSliderHandle_t aSliderNumber, uint16_t aBarBackgroundColor);

    void setSliderCaptionProperties(BDSliderHandle_t aSliderNumber, uint8_t aCaptionSize, uint8_t aCaptionPosition,
            uint8_t aCaptionMargin, Color_t aCaptionColor, Color_t aCaptionBackgroundColor);
    void setSliderCaption(BDSliderHandle_t aSliderNumber, const char * aCaption);

    void activateSlider(BDSliderHandle_t aSliderNumber);
    void deactivateSlider(BDSliderHandle_t aSliderNumber);
    void activateAllSliders(void);
    void deactivateAllSliders(void);

    struct XYSize mReferenceDisplaySize; // contains requested display size
    struct XYSize mActualDisplaySize;
    struct XYSize mMaxDisplaySize;
    uint32_t mHostUnixTimestamp;
    uint32_t mLocalMillisForHostTimestamp;

    volatile bool mConnectionEstablished;
    volatile bool mOrientationIsLandscape;

    /* for tests */
    void drawGreyscale(uint16_t aXPos, uint16_t tYPos, uint16_t aHeight);
    void drawStar(int aXPos, int aYPos, int tOffsetCenter, int tLength, int tOffsetDiagonal, int tLengthDiagonal, Color_t aColor);
    void testDisplay(void);
    void generateColorSpectrum(void);

private:
    uint16_t mActualDisplayHeight;
    uint16_t mActualDisplayWidth;
};

// The instance provided by the class itself
extern BlueDisplay BlueDisplay1;

#ifdef LOCAL_DISPLAY_EXISTS
#include <MI0283QT2.h>

/*
 * MI0283QT2 TFTDisplay - must provided by main program
 * external declaration saves ROM (210 Bytes) and RAM ( 20 Bytes)
 * and avoids missing initialization :-)
 */
extern MI0283QT2 LocalDisplay;
// to be provided by local display library
extern const unsigned int LOCAL_DISPLAY_HEIGHT;
extern const unsigned int LOCAL_DISPLAY_WIDTH;
#endif
// to be provided by another source (main.cpp)
extern const unsigned int REMOTE_DISPLAY_HEIGHT;
extern const unsigned int REMOTE_DISPLAY_WIDTH;

#endif // __cplusplus

extern bool isLocalDisplayAvailable;

#ifdef __cplusplus
extern "C" {
#endif
// for use in syscalls.c
uint16_t drawTextC(uint16_t aXStart, uint16_t aYStart, const char *aStringPtr, uint16_t aFontSize, Color_t aFGColor,
        Color_t aBGColor);
void writeStringC(const char *aStringPtr, uint8_t aStringLength);
#ifdef __cplusplus
}
#endif

/*
 * Utilities used also internal for
 */
#ifdef AVR
uint16_t getADCValue(uint8_t aChannel, uint8_t aReference);
float getVCCValue(void);
float getTemperature(void);
#endif

// for convenience also included here
#include "BlueSerial.h"
#include "EventHandler.h"

#endif /* BLUEDISPLAY_H_ */


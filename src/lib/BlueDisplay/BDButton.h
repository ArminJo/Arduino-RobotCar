/*
 * BDButton.h
 *
 *   SUMMARY
 *  Blue Display is an Open Source Android remote Display for Arduino etc.
 *  It receives basic draw requests from Arduino etc. over Bluetooth and renders it.
 *  It also implements basic GUI elements as buttons and sliders.
 *  GUI callback, touch and sensor events are sent back to Arduino.
 *
 *  Copyright (C) 2015  Armin Joachimsmeyer
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

#ifndef BLUEDISPLAY_INCLUDE_BDBUTTON_H_
#define BLUEDISPLAY_INCLUDE_BDBUTTON_H_

#include <stdint.h>

#ifdef AVR
#include <avr/pgmspace.h>
#endif

#ifdef LOCAL_DISPLAY_EXISTS
#include "TouchButton.h"
// since we have only a restricted pool of local buttons
typedef uint8_t BDButtonHandle_t;
#else
#ifdef AVR
typedef uint8_t BDButtonHandle_t;
#else
typedef uint16_t BDButtonHandle_t;
#endif
#endif

extern BDButtonHandle_t sLocalButtonIndex; // local button index counter used by BDButton.init() and BlueDisplay.createButton()

//#include "BlueDisplay.h" // for Color_t - cannot be included here since BlueDisplay.h needs BDButton
typedef uint16_t Color_t;

#ifdef __cplusplus
class BDButton {
public:

    static void resetAllButtons(void);
    static void activateAllButtons(void);
    static void deactivateAllButtons(void);
    static void setButtonsTouchTone(uint8_t aToneIndex, uint16_t aToneDuration);
    static void setButtonsTouchTone(uint8_t aToneIndex, uint16_t aToneDuration, uint8_t aToneVolume);
    static void setGlobalFlags(uint16_t aFlags);

    // Constructors
    BDButton();
    BDButton(BDButtonHandle_t aButtonHandle);
#ifdef LOCAL_DISPLAY_EXISTS
    BDButton(BDButtonHandle_t aButtonHandle, TouchButton * aLocalButtonPtr);
#endif
    BDButton(const BDButton &aButton);
    // Operators
    bool operator==(const BDButton &aButton);
    //
    bool operator!=(const BDButton &aButton);

    void init(uint16_t aPositionX, uint16_t aPositionY, uint16_t aWidthX, uint16_t aHeightY, Color_t aButtonColor,
            const char * aCaption, uint16_t aCaptionSize, uint8_t aFlags, int16_t aValue,
            void (*aOnTouchHandler)(BDButton*, int16_t));

    void drawButton(void);
    void removeButton(Color_t aBackgroundColor);
    void drawCaption(void);
    void setCaption(const char * aCaption);
    void setCaptionForValueTrue(const char * aCaption);
    void setCaptionAndDraw(const char * aCaption);
    void setCaption(const char * aCaption, bool doDrawButton);
    void setValue(int16_t aValue);
    void setValue(int16_t aValue, bool doDrawButton);
    void setValueAndDraw(int16_t aValue);
    void setButtonColor(Color_t aButtonColor);
    void setButtonColorAndDraw(Color_t aButtonColor);
    void setPosition(int16_t aPositionX, int16_t aPositionY);
    void setButtonAutorepeatTiming(uint16_t aMillisFirstDelay, uint16_t aMillisFirstRate, uint16_t aFirstCount,
            uint16_t aMillisSecondRate);

    void activate(void);
    void deactivate(void);

#ifdef AVR
    void initPGM(uint16_t aPositionX, uint16_t aPositionY, uint16_t aWidthX, uint16_t aHeightY, Color_t aButtonColor,
            const char * aPGMCaption, uint8_t aCaptionSize, uint8_t aFlags, int16_t aValue,
            void (*aOnTouchHandler)(BDButton*, int16_t));
    void setCaptionPGM(const char * aPGMCaption);
    void setCaptionPGMForValueTrue(const char * aCaption);
    void setCaptionPGM(const char * aPGMCaption, bool doDrawButton);
#endif

    BDButtonHandle_t mButtonHandle; // Index for BlueDisplay button functions

#ifdef LOCAL_DISPLAY_EXISTS
    void deinit(void);
    TouchButton * mLocalButtonPtr;
#endif

private:
};

#endif

#endif /* BLUEDISPLAY_INCLUDE_BDBUTTON_H_ */

/*
 * AutonomousDrivePage.cpp
 *
 *  Contains all the GUI elements for autonomous driving.
 *  Cont ->Step / Step -> SStep, SStep->Cont: Switches mode from "continuous drive" to "drive until next turn" to "drive CENTIMETER_PER_RIDE_PRO"
 *  Start Simple: Start simple driving algorithm (using the 2 "simple" functions in RobotCar.cpp)
 *  Start Pro: Start elaborated driving algorithm
 *
 *  Needs BlueDisplay library.
 *
 *  Copyright (C) 2019-2020  Armin Joachimsmeyer
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

BDButton TouchButtonStepMode;
BDButton TouchButtonStep;
BDButton TouchButtonSingleScan;
BDButton TouchButtonScanSpeed;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
BDButton TouchButtonScanMode;
#endif

BDButton TouchButtonTestUser;
BDButton TouchButtonBuiltInAutonomousDrive;

uint8_t sStepMode = MODE_CONTINUOUS;
bool sDoStep = false; // if true => do one step
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
uint8_t sScanMode = SCAN_MODE_BOTH;
#endif

bool sDoSlowScan = false;

bool sRuningAutonomousDrive = false;
bool sUseBuiltInAutonomousDriveStrategy = true;

void setStepModeButtonCaption();
/*
 * Switches modes MODE_CONTINUOUS -> MODE_STEP_TO_NEXT_TURN -> MODE_SINGLE_STEP
 */
void setStepMode(uint8_t aStepMode) {
    if (aStepMode == MODE_SINGLE_STEP) {
        RobotCar.stopCar();
    } else if (aStepMode > MODE_SINGLE_STEP) {
        aStepMode = MODE_CONTINUOUS;
    }
    sStepMode = aStepMode;
    setStepModeButtonCaption();
    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
        TouchButtonStepMode.drawButton();
    }
}

void doNextStepMode(BDButton * aTheTouchedButton, int16_t aValue) {
    sStepMode++;
    setStepMode(sStepMode);
}

/*
 * enables next step
 */
void doStep(BDButton * aTheTouchedButton, int16_t aValue) {
    if (sStepMode == MODE_CONTINUOUS) {
        // switch to step mode MODE_SINGLE_STEP
        setStepMode( MODE_SINGLE_STEP);
    }
    sDoStep = true;
    /*
     * Start if not yet done
     */
    if (!sRobotCarStarted) {
        startStopAutomomousDrive(true, sUseBuiltInAutonomousDriveStrategy);
    }
}

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
void setScanModeButtonCaption() {
    if (sScanMode == SCAN_MODE_BOTH) {
        TouchButtonScanMode.setCaption(F("Both->US"));
    } else if (sScanMode == SCAN_MODE_US)
#  ifdef CAR_HAS_IR_DISTANCE_SENSOR
    {
        TouchButtonScanMode.setCaption(F("US->IR"));
    } else {
        TouchButtonScanMode.setCaption(F("IR->Both"));
    }
# else
    {
        TouchButtonScanMode.setCaption(F("US->ToF"));
    } else {
        TouchButtonScanMode.setCaption(F("ToF->Both"));
    }
#  endif
}

void doScanMode(BDButton * aTheTouchedButton, int16_t aValue) {
    sScanMode++;
    if (sScanMode > SCAN_MODE_IR) {
        sScanMode = SCAN_MODE_BOTH;
    }
    setScanModeButtonCaption();
    TouchButtonScanMode.drawButton();
}

#endif // defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)

void doChangeScanSpeed(BDButton * aTheTouchedButton, int16_t aValue) {
    sDoSlowScan = aValue;
}

void doSingleScan(BDButton * aTheTouchedButton, int16_t aValue) {
    clearPrintedForwardDistancesInfos();
    fillAndShowForwardDistancesInfo(true, true);
    doWallDetection(true);
    sNextDegreesToTurn = doBuiltInCollisionDetection();
    drawCollisionDecision(sNextDegreesToTurn, CENTIMETER_PER_RIDE, false);
}

void startStopAutomomousDrive(bool aDoStart, bool aUseBuiltInAutonomousDriveStrategy) {
    sRuningAutonomousDrive = aDoStart;
    sUseBuiltInAutonomousDriveStrategy = aUseBuiltInAutonomousDriveStrategy;
    /*
     * Switch to right page. Needed for call from timeout condition.
     */
    if (sCurrentPage != PAGE_AUTOMATIC_CONTROL) {
        GUISwitchPages(NULL, PAGE_AUTOMATIC_CONTROL - sCurrentPage);
    }
    /*
     *  manage buttons
     */
    bool tInternalAutonomousDrive = false;
    bool tExternalAutonomousDrive = false;
    if (aDoStart) {
        resetPathData();
        sDoStep = true; // enable next step

        // decide which button enabled on start
        if (aUseBuiltInAutonomousDriveStrategy) {
            tInternalAutonomousDrive = true;
        } else {
            /*
             * Own test always starts in mode SINGLE_STEP
             */
            setStepMode(MODE_SINGLE_STEP);
            tExternalAutonomousDrive = true;
        }
    }
    TouchButtonBuiltInAutonomousDrive.setValueAndDraw(tInternalAutonomousDrive);
    TouchButtonTestUser.setValueAndDraw(tExternalAutonomousDrive);

    startStopRobotCar(aDoStart);
}

void doStartStopAutomomousDrive(BDButton * aTheTouchedButton, int16_t aValue) {
    startStopAutomomousDrive(aValue, true);
}

void doStartStopTestUser(BDButton * aTheTouchedButton, int16_t aValue) {
    startStopAutomomousDrive(aValue, false);
}

void setStepModeButtonCaption() {
    if (sStepMode == MODE_CONTINUOUS) {
        TouchButtonStepMode.setCaption(F("Continuous\n->\nStep to turn"));
    } else if (sStepMode == MODE_STEP_TO_NEXT_TURN) {
        TouchButtonStepMode.setCaption(F("Step to turn\n->\nSingle step"));
    } else {
        TouchButtonStepMode.setCaption(F("Single step\n->\nContinuous"));
    }
}

void initAutonomousDrivePage(void) {
    TouchButtonStepMode.init(0, 0, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_4, COLOR_BLUE, "", TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0,
            &doNextStepMode);
    setStepModeButtonCaption();

    TouchButtonSingleScan.init(0, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_4, COLOR_BLUE, F("Scan"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doSingleScan);

    TouchButtonScanSpeed.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR_BLACK, F("Scan slow"), TEXT_SIZE_16,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sDoSlowScan, &doChangeScanSpeed);
    TouchButtonScanSpeed.setCaptionForValueTrue("Scan fast");

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    TouchButtonScanMode.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR_RED, "", TEXT_SIZE_16, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doScanMode);
    setScanModeButtonCaption();
#endif

    TouchButtonStep.init(0, BUTTON_HEIGHT_4_LINE_3, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_4, COLOR_BLUE, F("Step"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doStep);

    TouchButtonBuiltInAutonomousDrive.init(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED,
            F("Start\nBuiltin"),
            TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            sRuningAutonomousDrive && sUseBuiltInAutonomousDriveStrategy, &doStartStopAutomomousDrive);
    TouchButtonBuiltInAutonomousDrive.setCaptionForValueTrue(F("Stop"));

    TouchButtonTestUser.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED,
            F("Start\nUser"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            sRuningAutonomousDrive && !sUseBuiltInAutonomousDriveStrategy, &doStartStopTestUser);
    TouchButtonTestUser.setCaptionForValueTrue(F("Stop\nUser"));

}

void drawAutonomousDrivePage(void) {
    drawCommonGui();

    BlueDisplay1.drawText(BUTTON_WIDTH_10_POS_4, TEXT_SIZE_22_HEIGHT + TEXT_SIZE_22_HEIGHT, F("Auto drive"));

    TouchButtonBackSmall.drawButton();

    TouchButtonStepMode.drawButton();
    TouchButtonSingleScan.drawButton();
    TouchButtonStep.drawButton();

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    TouchButtonScanMode.drawButton();
#endif
    TouchButtonScanSpeed.drawButton();

    TouchButtonBuiltInAutonomousDrive.drawButton();
    TouchButtonTestUser.drawButton();
    TouchButtonNextPage.drawButton();
}

void startAutonomousDrivePage(void) {
    TouchButtonTestUser.setValue(sRuningAutonomousDrive && !sUseBuiltInAutonomousDriveStrategy);
    TouchButtonBuiltInAutonomousDrive.setValue(sRuningAutonomousDrive && sUseBuiltInAutonomousDriveStrategy);
    setStepModeButtonCaption();

    TouchButtonBackSmall.setPosition(BUTTON_WIDTH_4_POS_4, 0);
    TouchButtonNextPage.setCaption(F("Show\nPath"));

    drawAutonomousDrivePage();
}

void loopAutonomousDrivePage(void) {
// nothing to do exclusively for this page
}

void stopAutonomousDrivePage(void) {
// no action needed
}

/*
 * Clear drawing area
 */
void clearPrintedForwardDistancesInfos() {
    BlueDisplay1.fillRectRel(US_DISTANCE_MAP_ORIGIN_X - US_DISTANCE_MAP_WIDTH_HALF,
    US_DISTANCE_MAP_ORIGIN_Y - US_DISTANCE_MAP_HEIGHT, US_DISTANCE_MAP_WIDTH_HALF * 2, US_DISTANCE_MAP_HEIGHT + 2, COLOR_WHITE);
}

/*
 * Draw values of ActualDistancesArray as vectors
 */
void drawForwardDistancesInfos() {
    color16_t tColor;
    uint8_t tActualDegrees = 0;
    /*
     * Clear drawing area
     */
    clearPrintedForwardDistancesInfos();
    for (int i = 0; i < NUMBER_OF_DISTANCES; ++i) {
        /*
         * Determine color
         */
        uint8_t tDistance = sForwardDistancesInfo.RawDistancesArray[i];
        tColor = COLOR_ORANGE;
        if (tDistance >= DISTANCE_TIMEOUT_CM) {
            tDistance = DISTANCE_TIMEOUT_CM;
            tColor = COLOR_GREEN;
        }
        if (tDistance > sCentimeterPerScanTimesTwo) {
            tColor = COLOR_GREEN;
        } else if (tDistance < sCentimeterPerScan) {
            tColor = COLOR_RED;
        }

        /*
         * Draw line
         */
        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tDistance, tActualDegrees, tColor, 3);
        tActualDegrees += DEGREES_PER_STEP;
    }
    doWallDetection(true);
}

void drawCollisionDecision(int aDegreeToTurn, uint8_t aLengthOfVector, bool aDoClear) {
    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
        color16_t tColor = COLOR_BLUE;
        int tDegreeToDisplay = aDegreeToTurn;
        if (tDegreeToDisplay == 180) {
            tColor = COLOR_PURPLE;
            tDegreeToDisplay = 0;
        }
        if (aDoClear) {
            tColor = COLOR_WHITE;
        }
        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, aLengthOfVector, tDegreeToDisplay + 90,
                tColor);
        if (!aDoClear) {
            sprintf_P(sStringBuffer, PSTR("wall%4d\xB0 rotation: %3d\xB0 wall%4d\xB0"), sForwardDistancesInfo.WallLeftAngleDegrees,
                    aDegreeToTurn, sForwardDistancesInfo.WallRightAngleDegrees); // \xB0 is degree character
            BlueDisplay1.drawText(US_DISTANCE_MAP_ORIGIN_X - US_DISTANCE_MAP_WIDTH_HALF, US_DISTANCE_MAP_ORIGIN_Y + TEXT_SIZE_11,
                    sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
        }
    }
}


/*
 * Distance.cpp
 *
 *  Contains all distance measurement functions.
 *
 *  Copyright (C) 2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include "Distance.h"
#include "RobotCar.h"
#include "RobotCarGui.h"

#include "HCSR04.h"

#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
SFEVL53L1X sToFDistanceSensor;
#endif

/*
 * This initializes the pins too
 */
void initDistance() {
#ifdef USE_US_SENSOR_1_PIN_MODE
    initUSDistancePin(PIN_TRIGGER_OUT);
#else
    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);
#endif
    initDistanceServo();

#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
    if (sToFDistanceSensor.begin() != 0) { //Begin returns 0 on a good init
        BlueDisplay1.debug("ToF sensor connect failed!");
    }
    // Short mode max distance is limited to 1.3 m but better ambient immunity. Above 1.3 meter we get error 4 (wrap around).
    sToFDistanceSensor.setDistanceModeShort();
    //sToFDistanceSensor.setDistanceModeLong(); // default
    sToFDistanceSensor.setOffset(OFFSET_MILLIMETER);

    /*
     * The minimum timing budget is 20 ms for short distance mode and 33 ms for medium and long distance modes.
     * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
     * This function must be called after SetDistanceMode.
     */
    sToFDistanceSensor.setTimingBudgetInMs(33);
#endif
}

void checkAndShowDistancePeriodically(uint16_t aPeriodMillis) {
    // Do not show distanced during (time critical) acceleration or deceleration
    if (!RobotCarMotorControl.needsFastUpdates()) {
        static long sLastUSMeasurementMillis;
        long tMillis = millis();
        if (sLastUSMeasurementMillis + aPeriodMillis < tMillis) {
            sLastUSMeasurementMillis = tMillis;
#ifdef CAR_HAS_IR_DISTANCE_SENSOR
            showIRDistance(getIRDistanceAsCentimeter());
#endif
#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
            showIRDistance(getToFDistanceAsCentimeter());
#endif
            // feedback as slider length
            showUSDistance(getUSDistanceAsCentiMeterWithCentimeterTimeout(300));
        }
    }
}

#ifdef CAR_HAS_IR_DISTANCE_SENSOR
uint8_t getIRDistanceAsCentimeter() {
    float tVolt = analogRead(PIN_IR_DISTANCE_SENSOR);
    // * 0.004887585 for 1023 = 5V
    // Model 1080
    return (29.988 * pow(tVolt * 0.004887585, -1.173)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp

    // Model 20150
//    return (60.374 * pow(tVolt * 0.004887585, -1.16)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
}
#endif

#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
/*
 * No start of measurement, just read result.
 */
uint8_t readToFDistanceAsCentimeter() {
    uint8_t i = 0; // for timeout
    // checkForDataReady needs 1.1 ms
    while (!sToFDistanceSensor.checkForDataReady() && i < 20) {
        delayAndLoopGUI(4);
        i++;
    }

    sToFDistanceSensor.clearInterrupt();
    uint8_t tStatus = sToFDistanceSensor.getRangeStatus();
    uint16_t tDistance = sToFDistanceSensor.getDistance(); //Get the result of the measurement from the sensor in millimeter
    if (sToFDistanceSensor.getRangeStatus() != 0) {
        if (tStatus == 4) {
            // Wrap around in mode short -> more than 130 cm
            tDistance = 1300;
        } else {

            tDistance = 10;
        }
    }
//    tone(SPEAKER_PIN, tDistance + 500);
    return tDistance / 10;
}

uint8_t getToFDistanceAsCentimeter() {
    sToFDistanceSensor.startRanging();
    return readToFDistanceAsCentimeter();
}

#endif

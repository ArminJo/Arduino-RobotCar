/*
 *  RobotCar.cpp
 *  Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
 *  To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the area.
 *  Manual control is by a GUI implemented with a Bluetooth HC-05 Module and the BlueDisplay library.
 *  Just overwrite the 2 functions myOwnFillForwardDistancesInfo() and doUserCollisionDetection() to test your own skill.
 *
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "RobotCar.h"

#include "HCSR04.h"

#define VERSION_EXAMPLE "1.0"

/*
 * Car Control
 */
CarMotorControl RobotCar;

/*
 * Start of robot car control program
 */
/*
 * Start of robot car control program
 */
void setup() {
// initialize digital pins as an output.
    pinMode(TRIGGER_OUT_PIN, OUTPUT);

#ifdef USE_US_SENSOR_1_PIN_MODE
    initUSDistancePin(TRIGGER_OUT_PIN);
#else
    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);
#endif

    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    EncoderMotor::enableINT0AndINT1Interrupts();
    EncoderMotor::EnableValuesPrint = true;

// initialize motors
    RobotCar.init(TWO_WD_DETECTION_PIN);

    // Just to know which program is running on my Arduino
    Serial.println("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);

}

void loop() {

    delay(100);
}


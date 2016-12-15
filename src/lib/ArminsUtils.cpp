/*
 *  ArminsUtils.cpp
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
#include "ArminsUtils.h"

// must not be constant, since then we get an undefined reference error at link time
extern uint8_t TRIGGER_OUT_PIN;
extern uint8_t ECHO_IN_PIN;

/*
 * Needed for NonBlocking pulseIn implementation
 * outcomment one of the following #defines if you need the interrupt vector for your own.
 */
#define USE_PIN_CHANGE_INTERRUPT_D0_TO_D7
#define USE_PIN_CHANGE_INTERRUPT_D8_TO_D13
#define USE_PIN_CHANGE_INTERRUPT_A0_TO_A5

#if defined (USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) || defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) || defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5)
volatile bool sUSValueIsValid = false;

unsigned long sMicrosAtStartOfPulse;
uint16_t sTimeoutMicros;
volatile unsigned long sMicrosOfPulse;
/*
 * This version only blocks for ca. 12 microseconds for code + generation of trigger pulse
 */
void getUSDistanceAsCentiMeterWithCentimeterTimeoutNonBlocking(uint8_t aTimeoutCentimeter) {
    // need minimum 10 usec Trigger Pulse
    digitalWrite(TRIGGER_OUT_PIN, HIGH);
    sUSValueIsValid = false;
    sTimeoutMicros = aTimeoutCentimeter * 59;
    *digitalPinToPCMSK(ECHO_IN_PIN) |= bit(digitalPinToPCMSKbit(ECHO_IN_PIN));  // enable pin
    PCICR |= bit(digitalPinToPCICRbit(ECHO_IN_PIN));  // enable interrupt for the group
    PCIFR |= bit(digitalPinToPCICRbit(ECHO_IN_PIN));  // clear any outstanding interrupt
    sMicrosOfPulse = 0;

#ifdef DEBUG
    delay(2); // to see it on scope
#else
    delayMicroseconds(10);
#endif
    // falling edge starts measurement
    digitalWrite(TRIGGER_OUT_PIN, LOW);
}

/*
 * If ISR interrupts these code, everything is fine, even if we get a timeout and a no null result
 * since we are interested in the result and not in very exact interpreting of the timeout.
 */
bool USDistanceIsMeasureFinished() {
    if (sUSValueIsValid) {
        return true;
    }
    if (micros() - sMicrosAtStartOfPulse >= sTimeoutMicros) {
        // Timeout happened value will be 0
        *digitalPinToPCMSK(ECHO_IN_PIN) &= ~(bit(digitalPinToPCMSKbit(ECHO_IN_PIN)));  // disable pin
        return true;
    }
    return false;
}
#endif

// Use one Routine to handle each group

#if defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13)
// handle pin change interrupt for D8 to D13 here
ISR (PCINT0_vect) {
    digitalWrite(13, digitalRead(8) and digitalRead(9));
}
#endif
#if defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5)
// handle pin change interrupt for A0 to A5 here
ISR (PCINT1_vect) {
    uint8_t tState = PINC && bit((digitalPinToPCMSKbit(ECHO_IN_PIN)));
    if (tState > 0) {
        sMicrosAtStartOfPulse = micros();
    } else {
        sMicrosOfPulse = micros() - sMicrosAtStartOfPulse;
        sUSValueIsValid = true;
    }
    digitalWrite(13, tState);
}
#endif
#if defined (USE_PIN_CHANGE_INTERRUPT_D0_TO_D7)
// handle pin change interrupt for D0 to D7 here
ISR (PCINT2_vect) {
    digitalWrite(13, digitalRead(7));
}
#endif

int sLastDistance;
unsigned int getUSDistanceAsCentiMeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter) {
// 58,48 us per centimeter (forth and back)
    // Must be the reciprocal of formula below
    unsigned int tTimeoutMicros = (aTimeoutCentimeter - 1) * 58;
    return getUSDistanceAsCentiMeter(tTimeoutMicros);
}
/*
 * returns aTimeoutMicros if timeout happens
 * timeout of 5850 is equivalent to 1m
 */
unsigned int getUSDistanceAsCentiMeter(unsigned int aTimeoutMicros) {
// need minimum 10 usec Trigger Pulse
    digitalWrite(TRIGGER_OUT_PIN, HIGH);
#ifdef DEBUG
    delay(2); // to see it on scope
#else
    delayMicroseconds(10);
#endif
// falling edge starts measurement
    digitalWrite(TRIGGER_OUT_PIN, LOW);

    /*
     * Get echo length. 58,48 us per centimeter (forth and back)
     * => 50cm gives 2900 us, 2m gives 11900 us
     */
    unsigned int tPulseLength = pulseIn(ECHO_IN_PIN, HIGH, aTimeoutMicros);
    if (tPulseLength == 0) {
        // timeout happened
        tPulseLength = aTimeoutMicros;
    }
// +1cm was measured at working device
    unsigned int tDistance = (tPulseLength / 58) + 1;
    sLastDistance = tDistance;
    return tDistance;
}

void blinkLed(uint8_t aLedPin, uint8_t aNumberOfBlinks, uint16_t aBlinkDelay) {
    for (int i = 0; i < aNumberOfBlinks; i++) {
        digitalWrite(aLedPin, HIGH);
        delay(aBlinkDelay);
        digitalWrite(aLedPin, LOW);
        delay(aBlinkDelay);
    }
}

/*
 * For sleep modes see sleep.h
 * SLEEP_MODE_IDLE
 * SLEEP_MODE_ADC
 * SLEEP_MODE_PWR_DOWN
 * SLEEP_MODE_PWR_SAVE
 * SLEEP_MODE_STANDBY
 * SLEEP_MODE_EXT_STANDBY
 */
void initSleep(uint8_t tSleepMode) {
    sleep_enable()
    ;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

/*
 * aWatchdogPrescaler (see wdt.h) can be one of
 * WDTO_15MS, 30, 60, 120, 250, WDTO_500MS
 * WDTO_1S to WDTO_8S
 * (value 0-9)
 */
void sleepWithWatchdog(uint8_t aWatchdogPrescaler) {
    MCUSR &= ~(1 << WDRF); // Clear WDRF in MCUSR
    cli();
    wdt_reset();
    WDTCSR |= (1 << WDCE) | (1 << WDE); // Bit 3+4 to unlock WDTCSR
    WDTCSR = aWatchdogPrescaler | (1 << WDIE) | (1 << WDIF); // Watchdog prescaler + interrupt enable + reset interrupt flag
    sei();
    sleep_cpu()
    ;
    wdt_disable();
}

volatile uint16_t sNumberOfSleeps = 0;

ISR(WDT_vect) {
    sNumberOfSleeps++;
}

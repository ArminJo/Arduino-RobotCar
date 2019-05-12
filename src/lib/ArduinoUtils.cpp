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

// Outcomment the line according to the ECHO_IN_PIN if using the non blocking version
//#define USE_PIN_CHANGE_INTERRUPT_D0_TO_D7  // using PCINT2_vect - PORT D
//#define USE_PIN_CHANGE_INTERRUPT_D8_TO_D13 // using PCINT0_vect - PORT B - Pin 13 is feedback output
//#define USE_PIN_CHANGE_INTERRUPT_A0_TO_A5  // using PCINT1_vect - PORT C

/*
 * This version only blocks for ca. 12 microseconds for code + generation of trigger pulse
 * Be sure to have the right interrupt vector below
 */
#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))
volatile bool sUSValueIsValid = false;
unsigned long sMicrosAtStartOfPulse;
uint16_t sTimeoutMicros;
volatile unsigned long sMicrosOfPulse;

// common code for all interrupt handler
void handlePCInterrupt(uint8_t atPortState) {
    if (atPortState > 0) {
        // start of pulse
        sMicrosAtStartOfPulse = micros();
    } else {
        // end of pulse
        sMicrosOfPulse = micros() - sMicrosAtStartOfPulse;
        sUSValueIsValid = true;
    }
    // echo to output 13
    digitalWrite(13, atPortState);
}
#endif

#if defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7)
/*
 * pin change interrupt for D0 to D7 here.
 * state of pin is echoed to output 13 for debugging purpose
 */
ISR (PCINT2_vect) {
    // check pin
    uint8_t tPortState = digitalPinToPort(ECHO_IN_PIN) && bit((digitalPinToPCMSKbit(ECHO_IN_PIN)));
    handlePCInterrupt(tPortState);
}
#endif

#if defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13)
/*
 * pin change interrupt for D8 to D13 here.
 * state of pin is echoed to output 13 for debugging purpose
 */
ISR (PCINT0_vect) {
    // check pin
    uint8_t tPortState = digitalPinToPort(ECHO_IN_PIN) && bit((digitalPinToPCMSKbit(ECHO_IN_PIN)));
    handlePCInterrupt(tPortState);
}
#endif

#if defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5)
/*
 * pin change interrupt for A0 to A5 here.
 * state of pin is echoed to output 13 for debugging purpose
 */
ISR (PCINT1_vect) {
    // check pin
    uint8_t tPortState = digitalPinToPort(ECHO_IN_PIN) && bit((digitalPinToPCMSKbit(ECHO_IN_PIN)));
    handlePCInterrupt(tPortState);
}
#endif

#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))

void getUSDistanceAsCentiMeterWithCentimeterTimeoutNonBlocking(uint8_t aTimeoutCentimeter) {
    // need minimum 10 usec Trigger Pulse
    digitalWrite(TRIGGER_OUT_PIN, HIGH);
    sUSValueIsValid = false;
    sTimeoutMicros = aTimeoutCentimeter * 59;
    *digitalPinToPCMSK(ECHO_IN_PIN) |= bit(digitalPinToPCMSKbit(ECHO_IN_PIN));// enable pin for pin change interrupt
    // net 2 registers exists only once!
    PCICR |= bit(digitalPinToPCICRbit(ECHO_IN_PIN));// enable interrupt for the group
    PCIFR |= bit(digitalPinToPCICRbit(ECHO_IN_PIN));// clear any outstanding interrupt
    sMicrosOfPulse = 0;

#ifdef DEBUG
    delay(2); // to see it on scope
#else
    delayMicroseconds(10);
#endif
    // falling edge starts measurement and generates first interrupt
    digitalWrite(TRIGGER_OUT_PIN, LOW);
}

/*
 * Used to check by polling.
 * If ISR interrupts these code, everything is fine, even if we get a timeout and a no null result
 * since we are interested in the result and not in very exact interpreting of the timeout.
 */
bool isUSDistanceIsMeasureFinished() {
    if (sUSValueIsValid) {
        return true;
    }
    if (micros() - sMicrosAtStartOfPulse >= sTimeoutMicros) {
        // Timeout happened value will be 0
        *digitalPinToPCMSK(ECHO_IN_PIN) &= ~(bit(digitalPinToPCMSKbit(ECHO_IN_PIN)));// disable pin for pin change interrupt
        return true;
    }
    return false;
}
#endif

/*
 * End non blocking implementation
 */

int sLastDistance;
unsigned int getUSDistanceAsCentiMeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter) {
// 58,48 us per centimeter (forth and back)
    // Must be the reciprocal of formula below
    unsigned int tTimeoutMicros = (aTimeoutCentimeter - 1) * 58;
    return getUSDistanceAsCentiMeter(tTimeoutMicros);
}
/*
 * returns aTimeoutMicros if timeout happens
 * timeout of 5850 micros is equivalent to 1m
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

#define COUNT_FOR_20_MILLIS 40000
/*
 * Use 16 bit timer1 for generating 2 servo signals entirely by hardware without any interrupts.
 * The 2 servo signals are tied to pin 9 and 10 of an 328.
 * Attention - both pins are set to OUTPUT here!
 */
void initSimpleServoPin9_10() {
    /*
     * Periods below 20 ms gives problems with long signals i.e. the positioning is not possible
     */
    DDRB |= _BV(DDB1) | _BV(DDB2);                // set pins OC1A = PortB1 -> PIN 9 and OC1B = PortB2 -> PIN 10 to output direction
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); // FastPWM Mode mode TOP determined by ICR1 - non-inverting Compare Output mode
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);    // set prescaler to 8, FastPWM Mode mode continued
    ICR1 = COUNT_FOR_20_MILLIS;      // set period to 20 ms
    OCR1A = 3000;      // set count to 1500 us - 90 degree
    OCR1B = 3000;      // set count to 1500 us - 90 degree
    TCNT1 = 0;         // reset timer
}

/*
 * If value is below 180 then assume degree, otherwise assume microseconds
 * UpdateFast -> if more than 5 ms since last pulse -> start a new one
 */
void setSimpleServoPulse(int aValue, bool aUsePin9, bool aUpdateNormal) {
    if (aValue <= 180) {
        //aValue = map(aValue, 0, 180, 1000, 5200); // values for a SG90 MicroServo
        aValue = map(aValue, 0, 180, 1088, 4800); // values compatible with standard arduino values
    } else {
        // since the resolution is 1/2 of microsecond
        aValue *= 2;
    }
    if (!aUpdateNormal) {
        uint16_t tTimerCount = TCNT1;
        if (tTimerCount > 10000) {
            // more than 5 ms since last pulse -> start a new one
            TCNT1 = COUNT_FOR_20_MILLIS - 1;
        }
    }
    if (aUsePin9) {
        OCR1A = aValue;
    } else {
        OCR1B = aValue;
    }
}

/*
 * Pin 9 / Channel A. If value is below 180 then assume degree, otherwise assume microseconds
 */
void setSimpleServoPulsePin9(int aValue) {
    setSimpleServoPulse(aValue, true, false);
}

/*
 * Pin 10 / Channel B
 */
void setSimpleServoPulsePin10(int aValue) {
    setSimpleServoPulse(aValue, false, false);
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

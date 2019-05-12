#include <avr/sleep.h>
#include <avr/wdt.h>

#define US_DISTANCE_DEFAULT_TIMEOUT 20000
// Timeout of 20000L is 3.4 meter
unsigned int getUSDistanceAsCentiMeter(unsigned int aTimeoutMicros = US_DISTANCE_DEFAULT_TIMEOUT);
unsigned int getUSDistanceAsCentiMeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter);
extern int sLastDistance;

/*
 * Non blocking version
 */
void getUSDistanceAsCentiMeterWithCentimeterTimeoutNonBlocking(uint8_t aTimeoutCentimeter);
bool isUSDistanceIsMeasureFinished();

/*
 * Simple Servo Library
 * Uses timer1 and Pin 9 + 10 as Output
 */
void initSimpleServoPin9_10();
void setSimpleServoPulse(int aValue, bool aUsePin9, bool aUpdateNormal);
void setSimpleServoPulsePin9(int aValue); // Channel A
void setSimpleServoPulsePin10(int aValue); // Channel B


void initSleep(uint8_t tSleepMode);
void sleepWithWatchdog(uint8_t aWatchdogPrescaler);
extern volatile uint16_t sNumberOfSleeps;

void blinkLed(uint8_t aLedPin, uint8_t aNumberOfBlinks, uint16_t aBlinkDelay);

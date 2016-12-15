#include <avr/sleep.h>
#include <avr/wdt.h>

#define US_DISTANCE_DEFAULT_TIMEOUT 20000
// Timeout of 20000L is 3.4 meter
unsigned int getUSDistanceAsCentiMeter(unsigned int aTimeoutMicros = US_DISTANCE_DEFAULT_TIMEOUT);
unsigned int getUSDistanceAsCentiMeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter);
extern int sLastDistance;

void initSleep(uint8_t tSleepMode);
void sleepWithWatchdog(uint8_t aWatchdogPrescaler);
extern volatile uint16_t sNumberOfSleeps;

void blinkLed(uint8_t aLedPin, uint8_t aNumberOfBlinks, uint16_t aBlinkDelay);

// TypeDef.h

#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <stdint.h>

#define SINE_SAMPLES 256

// SineWave struct contains the sine table and parameters related to sine wave generation
typedef struct {
    uint16_t SineTable[SINE_SAMPLES];  // Store the sine wave samples
    uint16_t zoneThresholds[9];        // Zone thresholds for ADC values
    uint16_t prevZone;                 // The previous zone (for changes in the sine wave)
    volatile uint16_t beepRequest;     // Request to generate a beep sequence
} SineWave;

// Timer struct for managing timing-related variables
typedef struct {
    uint16_t delayValue;  // Delay value for the sensor polling (in ms)
    uint16_t raw;         // Raw ADC value from the sensor
    uint16_t currTime;    // Current time (for timing calculations)
    uint16_t prevTime;    // Previous time (for timing calculations)
} Timer;

// Function prototypes
void SilenceSineTable(SineWave *sineWave);
void UpdateSineTable(SineWave *sineWave, uint16_t amplitude);
void updateLcdZone(uint16_t zone);
int GetZone(uint16_t adcValue);

#endif // TYPEDEF_H

#ifndef AEDB_9140_H
#define AEDB_9140_H
/*
    AEDB-9140 Encoder Header File
    Created By Samuel Walton
    Editted by Caleb Evans
    The University of Manchester
    Steps for use on Nordic NRF52833 Board
    1. Add Inputs for three pins in Device Trees in Overlay file
    2. Add this header file to target sources in "CMakeLists.txt" as
   "target_sources(app PRIVATE src/AEDB-9140.h)"
    3. Include this header file in main.c

*/
#include <zephyr/kernel.h>

#define ENCODER_SAMPLE_PERIOD_MS 100

int32_t getIntVel(void);
float getFloatVel(void);
int32_t getIntAcc(void);
float getFloatAcc(void);
int32_t getPosition(void);
void resetPosition(int32_t in);

void Setup_interrupt(void);
void encoderTestLoop(void);

// Should be called every period.
void setVelocity(void);
// Should be called every 3rd period.
void setAcceleration(void);

#endif

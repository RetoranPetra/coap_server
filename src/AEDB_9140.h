#ifndef AEDB_9140_H
#define AEDB_9140_H
/*
    AEDB-9140 Encoder Header File
    Created By Samuel Walton 
    The University of Manchester
    Steps for use on Nordic NRF52833 Board
    1. Add Inputs for three pins in Device Trees in Overlay file
    2. Add this header file to target sources in "CMakeLists.txt" as "target_sources(app PRIVATE src/AEDB-9140.h)"
    3. Include this header file in main.c

*/
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/device.h>

#define sample_period 1E-2

static const struct gpio_dt_spec ChannelA_Encoder = GPIO_DT_SPEC_GET(DT_NODELABEL(encodercha), gpios);
static const struct gpio_dt_spec ChannelB_Encoder = GPIO_DT_SPEC_GET(DT_NODELABEL(encoderchb), gpios);
static const struct gpio_dt_spec ChannelI_Encoder = GPIO_DT_SPEC_GET(DT_NODELABEL(encoderindex), gpios);

static struct gpio_callback encoderA_callback;
static struct gpio_callback encoderB_callback;
static struct gpio_callback encoderI_callback;

static const int QEM [4][4] = {{0,-1,1,2},{1,0,2,-1},{-1,2,0,1},{2,1,-1,0}}; // Quadrature Encoder Matrix
int newState = 0;
int oldState = 0;
int32_t position =0;
int32_t position_prevfullrev = 0;
int32_t FullResolutionPositionCount = 0;
int32_t intVel = 0;
float floatVel = 0.0f;
int32_t intAcc = 0;
float floatAcc = 0.0f;

int32_t getIntVel(void) {
  return intVel;
}
float getFloatVel(void) {
  return floatVel;
}
int32_t getIntAcc(void) {
  return intAcc;
}
float getFloatAcc(void) {
  return floatAcc;
}

void setPosition()
{
    oldState = newState;
    newState = (gpio_pin_get_dt(&ChannelB_Encoder) << 1) + gpio_pin_get_dt(&ChannelA_Encoder);
    position += QEM[newState][oldState];
    //return newState, oldState, position;
}

void FullRotationCounter()
{
    FullResolutionPositionCount = position - position_prevfullrev;
    position_prevfullrev = position;
    //printf("Full Rotation Counter: %d",FullResolutionPositionCount);
    //return FullResolutionPositionCount, position_prev;
}

void Setup_interrupt(void)
{
    
    gpio_pin_interrupt_configure_dt(&ChannelA_Encoder,GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&ChannelB_Encoder,GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&ChannelI_Encoder,GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&encoderA_callback, setPosition, BIT(ChannelA_Encoder.pin)); 	
    gpio_add_callback(ChannelA_Encoder.port, &encoderA_callback);
    gpio_init_callback(&encoderB_callback, setPosition, BIT(ChannelB_Encoder.pin)); 	
    gpio_add_callback(ChannelB_Encoder.port, &encoderB_callback);
    gpio_init_callback(&encoderI_callback, FullRotationCounter, BIT(ChannelI_Encoder.pin)); 	
    gpio_add_callback(ChannelI_Encoder.port, &encoderI_callback);
}

int getPosition(void) {
    return position;
}

//Should be called every period.
void setVelocity(void) {
  static int32_t previousPosition = 0;
  intVel = position-previousPosition;
  floatVel = (float)intVel*(40.85E-6)/(float)sample_period;
  previousPosition = position;
}
//Should be called every 3rd period.
void setAcceleration(void) {
  static int32_t previousVelocity = 0;
  static float previousFloatVel = 0;
  intAcc = intVel - previousVelocity;
  floatAcc = (floatVel-previousFloatVel)/(float)sample_period/3.0f;
  previousVelocity = intVel;
  previousFloatVel = floatVel;
}

#endif

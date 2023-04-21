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

#define sample_period 1E-3

static const struct gpio_dt_spec ChannelA_Encoder = GPIO_DT_SPEC_GET(DT_NODELABEL(encodercha), gpios);
static const struct gpio_dt_spec ChannelB_Encoder = GPIO_DT_SPEC_GET(DT_NODELABEL(encoderchb), gpios);
static const struct gpio_dt_spec ChannelI_Encoder = GPIO_DT_SPEC_GET(DT_NODELABEL(encoderindex), gpios);

static struct gpio_callback encoderA_callback;
static struct gpio_callback encoderB_callback;
static struct gpio_callback encoderI_callback;

static const int QEM [4][4] = {{0,-1,1,2},{1,0,2,-1},{-1,2,0,1},{2,1,-1,0}}; // Quadrature Encoder Matrix
int newState = 0;
int oldState = 0;
int Position_counter =0;
int position_prevfullrev = 0;
int FullResolutionPositionCount = 0;
double velocity = 0;
double acceleration = 0;
double prev_position = 0;
double prev_velocity = 0;

void getAEBD9140Velocity()
{
    prev_position = Position_counter;
    prev_velocity = velocity;
    oldState = newState;
    newState = (gpio_pin_get_dt(&ChannelB_Encoder) << 1) + gpio_pin_get_dt(&ChannelA_Encoder);
    Position_counter += QEM[newState][oldState];
    velocity = (Position_counter-prev_position)*(40.85E-6)/sample_period;
    acceleration = (velocity-prev_velocity)/sample_period;
    //return newState, oldState, Position_counter;
}

void FullRotationCounter()
{
    FullResolutionPositionCount = Position_counter - position_prevfullrev;
    position_prevfullrev = Position_counter;
    //printf("Full Rotation Counter: %d",FullResolutionPositionCount);
    //return FullResolutionPositionCount, position_prev;
}

void Setup_interrupt()
{
    
    gpio_pin_interrupt_configure_dt(&ChannelA_Encoder,GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&ChannelB_Encoder,GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&ChannelI_Encoder,GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&encoderA_callback, getAEBD9140Velocity, BIT(ChannelA_Encoder.pin)); 	
    gpio_add_callback(ChannelA_Encoder.port, &encoderA_callback);
    gpio_init_callback(&encoderB_callback, getAEBD9140Velocity, BIT(ChannelB_Encoder.pin)); 	
    gpio_add_callback(ChannelB_Encoder.port, &encoderB_callback);
    gpio_init_callback(&encoderI_callback, FullRotationCounter, BIT(ChannelI_Encoder.pin)); 	
    gpio_add_callback(ChannelI_Encoder.port, &encoderI_callback);
}

int get_position_counter(void) {
    return Position_counter;
}
#endif

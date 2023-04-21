#include "AEDB_9140.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include <zephyr/device.h>

LOG_MODULE_REGISTER(encoder, CONFIG_ENCODER_LOG_LEVEL);
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
// TODO: Hold setPosition accountable to FullRotationCounter,
// prioritise FullRotationCounter readings over setposition ones, as will be
// more accurate due to less interrupts.

// NOTE: Check if splitting set position so it doesn't rely on gpio_checks makes
// it work better, e.g. one func for up on A, one for done on A, same for B etc.
// Don't seem to be able to have two seperate callbacks for one pin.

// NOTE: Reimplement callbacks to be specific to the callback type. Seems to
// take in an integer, which might be up/down information? Documentation
// suggests it's just an inbetween, doesn't actually give info to the function.

//TODO: only check GPIO on respective callback, not both.
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

//TODO: Call this automatically with the simple timer module. https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.2.0%2Fgroup__app__simple__timer__config.html

//Should be called every period.
void setVelocity(void) {
  static int32_t previousPosition = 0;
  intVel = position-previousPosition;
  floatVel = (float)intVel*(40.85E-6)/(float)ENCODER_SAMPLE_PERIOD;
  previousPosition = position;
}
//Should be called every 3rd period.
void setAcceleration(void) {
  static int32_t previousVelocity = 0;
  static float previousFloatVel = 0;
  intAcc = intVel - previousVelocity;
  floatAcc = (floatVel-previousFloatVel)/(float)ENCODER_SAMPLE_PERIOD/3.0f;
  previousVelocity = intVel;
  previousFloatVel = floatVel;
}
